#pragma once

#include <foundation/newton_raphson.h>
#include <foundation/types.h>

#include <iostream>

enum class ProjectionType { PERSPECTIVE, BROWN, FISHEYE, SPHERICAL, DUAL };
enum class Disto { K1 = 0, K2 = 1, K3 = 2, P1 = 3, P2 = 4, COUNT = 5 };

template <class T>
inline T SquaredNorm(T* point){
  return point[0]*point[0] + point[1]*point[1];
}

/* Parameters are : none used */
struct FisheyeProjection {
  template <class T>
  static void Forward(const T* point, const T* /* p */, T* projected) {
    const T r = sqrt(SquaredNorm(point));
    const auto theta = atan2(r, point[2]);
    projected[0] = theta / r * point[0];
    projected[1] = theta / r * point[1];
  }

  template <class T>
  static void Backward(const T* point, const T* /* p */, T* bearing) {
    const T theta = sqrt(SquaredNorm(point));
    const auto s = tan(theta) / theta;
    const T x = s * point[0];
    const T y = s * point[1];
    const T inv_norm = T(1.0) / sqrt(x*x + y*y + T(1.0));
    bearing[0] = point[0] * inv_norm;
    bearing[1] = point[1] * inv_norm;
    bearing[2] = inv_norm;
  }
};

/* Parameters are : none used */
struct PerspectiveProjection {
  template <class T>
  static void Forward(const T* point, const T* /* p */, T* projected) {
    projected[0] = point[0] / point[2];
    projected[1] = point[1] / point[2];
  }

  template <class T>
  static void Backward(const T* point, const T* /* p */, T* bearing) {
    const T inv_norm = T(1.0) / sqrt(SquaredNorm(point) + T(1.0));
    bearing[0] = point[0] * inv_norm;
    bearing[1] = point[1] * inv_norm;
    bearing[2] = inv_norm;
  }
};

/* Parameters are : transition */
struct DualProjection {
  template <class T>
  static void Forward(const T* point, const T* p, T* projected) {
    T p_persp[2];
    PerspectiveProjection::Forward(point, p, p_persp);
    T p_fish[2];
    FisheyeProjection::Forward(point, p, p_fish);
    projected[0] = p[0] * p_persp[0] + (1.0 - p[0]) * p_fish[0];
    projected[1] = p[0] * p_persp[1] + (1.0 - p[0]) * p_fish[1];
  }

  template <class T>
  static void Backward(const T* point, const T* p, T* bearing) {
    // Perform a bit iterations for finding theta from r
    const T r = sqrt(SquaredNorm(point));
    ThetaEval<T> eval_function{0, r, p[0]};
    const auto theta_refined =
        NewtonRaphson<ThetaEval<T>, 1, 1, ManualDiff<ThetaEval<T>, 1, 1>>(
            eval_function, 0, iterations);

    const auto s = tan(theta_refined) /
                   (p[0] * tan(theta_refined) + (1.0 - p[0]) * theta_refined);

    const T x = s * point[0];
    const T y = s * point[1];
    const T inv_norm = T(1.0) / sqrt(x*x + y*y + T(1.0));
    bearing[0] = point[0] * inv_norm;
    bearing[1] = point[1] * inv_norm;
    bearing[2] = inv_norm;
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

/* Parameters are : none used */
struct SphericalProjection {
  template <class T>
  static void Forward(const T* point, const T* /* p */, T* projected) {
    const auto lon = atan2(point[0], point[2]);
    const auto lat = atan2(-point[1], hypot(point[0], point[2]));
    projected[0] = lon / (2 * M_PI);
    projected[1] = -lat / (2 * M_PI);
  }

  template <class T>
  static void Backward(const T* point, const T* /* p */, T* bearing) {
    const auto lon = point[0] * 2 * M_PI;
    const auto lat = -point[1] * 2 * M_PI;
    bearing[0] = cos(lat) * sin(lon);
    bearing[1] = -sin(lat);
    bearing[2] = cos(lat) * cos(lon);
  }
};

/* Parameters are : k1, k2 */
struct Disto24 {
  template <class T>
  static void Forward(const T* point, const T* k, T* distorted) {
    const T r2 = point[0]*point[0] + point[1]*point[1];
    const auto distortion = Distortion(r2, k[static_cast<int>(Disto::K1)],
                                       k[static_cast<int>(Disto::K2)]);
    distorted[0] = point[0] * distortion;
    distorted[1] = point[1] * distortion;
  }

  template <class T>
  static void Backward(const T* point, const T* k, T* undistorted) {
    /* Beware if you use Backward together with autodiff. You'll need to remove
     * the line below, otherwise, derivatives won't be propagated */
    const T rd = sqrt(SquaredNorm(point));
    if (rd < T(std::numeric_limits<double>::epsilon())) {
      undistorted[0] = point[0];
      undistorted[1] = point[1];
    }

    // Compute undistorted radius
    DistoEval<T> eval_function{rd, k[static_cast<int>(Disto::K1)],
                               k[static_cast<int>(Disto::K2)]};
    const auto ru_refined =
        NewtonRaphson<DistoEval<T>, 1, 1, ManualDiff<DistoEval<T>, 1, 1>>(
            eval_function, rd, iterations);

    // Compute distortion factor from undistorted radius
    const T r2 = ru_refined * ru_refined;
    const auto distortion = Distortion(r2, k[static_cast<int>(Disto::K1)],
                                       k[static_cast<int>(Disto::K2)]);

    // Unapply undistortion
    undistorted[0] = point[0] / distortion;
    undistorted[1] = point[1] / distortion;
  }

  static constexpr int iterations = 10;
  template <class T>
  struct DistoEval {
    const T& rd;
    const T& k1;
    const T& k2;
    T operator()(const T& x) const {
      const auto r2 = x * x;
      return x * Disto24::Distortion(r2, k1, k2) - rd;
    }
    T derivative(const T& x) const {
      const auto r2 = x * x;
      return Disto24::DistortionDerivative(r2, k1, k2);
    }
  };

  template <class T>
  static inline T Distortion(const T& r2, const T& k1, const T& k2) {
    return T(1.0) + r2 * (k1 + k2 * r2);
  }

  template <class T>
  static T DistortionDerivative(const T& r2, const T& k1, const T& k2) {
    return T(1.0) + r2 * T(2.0) * (k1 + T(2.0) * k2 * r2);
  }
};

/* Parameters are : k1, k2, k3, p1, p2 */
struct DistoBrown {
  template <class T>
  static void Forward(const T* point, const T* k, T* distorted) {
    const auto r2 = SquaredNorm(point);
    const auto distortion_radial = RadialDistortion(
        r2, k[static_cast<int>(Disto::K1)], k[static_cast<int>(Disto::K2)],
        k[static_cast<int>(Disto::K3)]);
    const auto distortion_tangential = TangentialDistortion(
        r2, point[0], point[1], k[static_cast<int>(Disto::P1)],
        k[static_cast<int>(Disto::P2)]);
    distorted[0] = point[0] * distortion_radial + distortion_tangential[0];
    distorted[1] = point[1] * distortion_radial + distortion_tangential[1];
  }

  template <class T>
  static void Backward(const T* point, const T* k, T* undistorted) {
    /* Beware if you use Backward together with autodiff. You'll need to remove
     * the line below, otherwise, derivatives won't be propagated */
    const T rd = sqrt(SquaredNorm(point));
    if (rd < T(std::numeric_limits<double>::epsilon())) {
      undistorted[0] = point[0];
      undistorted[1] = point[1];
    }

    Eigen::Map<const Vec2<T>> mapped_point(point);
    DistoEval<T> eval_function{mapped_point,
                               k[static_cast<int>(Disto::K1)],
                               k[static_cast<int>(Disto::K2)],
                               k[static_cast<int>(Disto::K3)],
                               k[static_cast<int>(Disto::P1)],
                               k[static_cast<int>(Disto::P2)]};
    Eigen::Map<Vec2<T>> mapped_undistorted(undistorted);
    mapped_undistorted =
        NewtonRaphson<DistoEval<T>, 2, 2, ManualDiff<DistoEval<T>, 2, 2>>(
            eval_function, mapped_point, iterations);
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
      const T r2 = point.squaredNorm();
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

/* Parameters are : focal, aspect ratio, cx, cy */
struct Affine {
  template <class T>
  static void Forward(const T* point, const T* affine, T* transformed) {
    transformed[0] = affine[0] * point[0] + affine[2];
    transformed[1] = affine[0] * affine[1] * point[1] + affine[3];
  }

  template <class T>
  static void Backward(const T* point, const T* affine, T* transformed) {
    transformed[0] = (point[0] - affine[2])/affine[0];
    transformed[1] = (point[1] - affine[3])/(affine[1]*affine[0]);
  }
};

/* Parameters are : focal */
struct UniformScale {
  template <class T>
  static void Forward(const T* point, const T* scale, T* transformed) {
    transformed[0] = scale[0] * point[0];
    transformed[1] = scale[0] * point[1];
  }

  template <class T>
  static void Backward(const T* point, const T* scale, T* transformed) {
    transformed[0] = point[0] / scale[0];
    transformed[1] = point[1] / scale[0];
  }
};

/* Parameters are : none used */
struct Identity {
  template <class T>
  static void Forward(const T* point, const T* /* k */, T* transformed) {
    transformed[0] = point[0];
    transformed[1] = point[1];
  }

  template <class T>
  static void Backward(const T* point, const T* /* k */, T* transformed) {
    transformed[0] = point[0];
    transformed[1] = point[1];
  }
};

struct ProjectFunction {
  template <class TYPE, class T>
  static void Apply(const T* point, const T* parameters, T* projected) {
    TYPE::Forward(point, parameters, projected);
  }
};

struct BearingFunction {
  template <class TYPE, class T>
  static void Apply(const T* point, const T* parameters, T* bearing) {
    TYPE::Backward(point, parameters, bearing);
  }
};

/* This struct helps define most cameras models as they tend to follow the
 * pattern PROJ - > DISTO -> AFFINE. However, its is not mandatory for any
 * camera model to follow it. You can add any new camera models as long as it
 * implements the Forward and Backward functions. */

/* Here's some trait that defines where to look for parameters that follows the
 * generic scheme */
template <class PROJ, class DISTO, class AFF>
struct ParametersIndexTraits {};
template <>
struct ParametersIndexTraits<PerspectiveProjection, Disto24, UniformScale> {
  static constexpr int Projection = 0;  /* Unused in that case */
  static constexpr int Distorsion = 1;
  static constexpr int Affine = 0;
};
template <>
struct ParametersIndexTraits<FisheyeProjection, Disto24, UniformScale> {
  static constexpr int Projection = 0;  /* Unused in that case */
  static constexpr int Distorsion = 1;
  static constexpr int Affine = 0;
};
template <>
struct ParametersIndexTraits<DualProjection, Disto24, UniformScale> {
  static constexpr int Projection = 3;
  static constexpr int Distorsion = 1;
  static constexpr int Affine = 0;
};
template <>
struct ParametersIndexTraits<PerspectiveProjection, DistoBrown, Affine> {
  static constexpr int Projection = 0;  /* Unused in that case */
  static constexpr int Distorsion = 4;
  static constexpr int Affine = 0;
};
template <>
struct ParametersIndexTraits<SphericalProjection, Identity, Identity> {
  static constexpr int Projection = 0; /* Unused in that case */
  static constexpr int Distorsion = 0; /* Unused in that case */
  static constexpr int Affine = 0;     /* Unused in that case */
};

template <class PROJ, class DISTO, class AFF>
struct ProjectGeneric {
  using Indexes = ParametersIndexTraits<PROJ, DISTO, AFF>;

  template <class T>
  static void Forward(const T* point, const T* parameters, T* projected) {
    PROJ::Forward(point, parameters + Indexes::Projection, projected);
    DISTO::Forward(projected, parameters + Indexes::Distorsion, projected);
    AFF::Forward(projected, parameters + Indexes::Affine, projected);
  };

  template <class T>
  static void Backward(const T* point, const T* parameters, T* bearing) {
    T tmp[2];
    AFF::Backward(point, parameters + Indexes::Projection, tmp);
    DISTO::Backward(tmp, parameters + Indexes::Distorsion, tmp);
    PROJ::Backward(tmp, parameters + Indexes::Affine, bearing);
  }
};

using PerspectiveCameraT = ProjectGeneric<PerspectiveProjection, Disto24, UniformScale>;
using BrownCameraT = ProjectGeneric<PerspectiveProjection, DistoBrown, Affine>;
using FisheyeCameraT = ProjectGeneric<FisheyeProjection, Disto24, UniformScale>;
using DualCameraT = ProjectGeneric<DualProjection, Disto24, UniformScale>;
using SphericalCameraT = ProjectGeneric<SphericalProjection, Identity, Identity>;

/* This is where the pseudo-strategy pattern takes place. If you want to add
 * your own new camera model, just add a new enum value, the corresponding
 * case below and the implementation (see above). */
template <class FUNC, class... IN>
void Dispatch(const ProjectionType& type, IN&&... args) {
  switch (type) {
    case ProjectionType::PERSPECTIVE:
      FUNC::template Apply<PerspectiveCameraT>(std::forward<IN>(args)...);
      break;
    case ProjectionType::BROWN:
      FUNC::template Apply<BrownCameraT>(std::forward<IN>(args)...);
      break;
    case ProjectionType::FISHEYE:
      FUNC::template Apply<FisheyeCameraT>(std::forward<IN>(args)...);
      break;
    case ProjectionType::DUAL:
      FUNC::template Apply<DualCameraT>(std::forward<IN>(args)...);
      break;
    case ProjectionType::SPHERICAL:
      FUNC::template Apply<SphericalCameraT>(std::forward<IN>(args)...);
      break;
    default:
      throw std::runtime_error("Invalid ProjectionType");
  }
};