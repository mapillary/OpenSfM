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

template< int IN, int P, int OUT>
struct CameraFunctor{
  constexpr static int InSize = IN;
  constexpr static int ParamSize = P;
  constexpr static int OutSize = OUT;
  template <bool C>
  static constexpr int Stride(){return C * ParamSize + InSize;};
};

/* Parameters are : none used */
struct FisheyeProjection : CameraFunctor<3, 0, 2>{
  template <class T>
  static void Forward(const T* point, const T* /* p */, T* projected) {
    const T r = sqrt(SquaredNorm(point));
    const auto theta = atan2(r, point[2]);
    projected[0] = theta / r * point[0];
    projected[1] = theta / r * point[1];
  }

  template <class T, bool COMP_PARAM>
  static void ForwardDerivatives(const T* point, const T* /* p */, T* projected,
                                 T* jacobian) {
    // dx, dy, dz
    constexpr int Stride = COMP_PARAM*ParamSize + InSize;
    const T r2 = SquaredNorm(point);
    const T r = sqrt(r2);
    const T R2 = r2 + point[2] * point[2];
    const T theta = atan2(r, point[2]);
    const T x2 = point[0]*point[0];
    const T y2 = point[1]*point[1];
    const T z2 = point[2]*point[2];

    const T inv_denom = T(1.0) / (r2 * R2 * r);
    jacobian[0] = (x2 * y2 * theta + y2 * y2 * theta + y2 * z2 * theta +
                   x2 * point[2] * r) *
                  inv_denom;
    jacobian[1] = point[0] * (point[1] * point[2] * r - point[1] * theta * R2) *
                  inv_denom;
    jacobian[2] = -point[0] / R2;
    jacobian[Stride] = point[1] *
                       (point[0] * point[2] * r - point[0] * theta * R2) *
                       inv_denom;
    jacobian[Stride + 1] = (x2 * y2 * theta + x2 * x2 * theta +
                            x2 * z2 * theta + y2 * point[2] * r) *
                           inv_denom;
    jacobian[Stride + 2] = -point[1] / R2;

    T* dummy = nullptr;
    Forward(point, dummy, projected);
  }

  template <class T>
  static void Backward(const T* point, const T* /* p */, T* bearing) {
    const T theta = sqrt(SquaredNorm(point));
    const auto s = tan(theta) / theta;
    const T x = s * point[0];
    const T y = s * point[1];
    const T inv_norm = T(1.0) / sqrt(x*x + y*y + T(1.0));
    bearing[0] = x * inv_norm;
    bearing[1] = y * inv_norm;
    bearing[2] = inv_norm;
  }
};

/* Parameters are : none used */
struct PerspectiveProjection : CameraFunctor<3, 0, 2> {
  template <class T>
  static void Forward(const T* point, const T* /* p */, T* projected) {
    projected[0] = point[0] / point[2];
    projected[1] = point[1] / point[2];
  }

  template <class T, bool COMP_PARAM>
  static void ForwardDerivatives(const T* point, const T* /* p */, T* projected,
                                 T* jacobian) {
    // dx, dy, dz
    constexpr int Stride = COMP_PARAM*ParamSize + InSize;
    jacobian[0] = T(1.0) / point[2];
    jacobian[1] = T(0.0);
    jacobian[2] = -point[0] / (point[2] * point[2]);
    jacobian[Stride] = T(0.0);
    jacobian[Stride + 1] = jacobian[0];
    jacobian[Stride + 2] = -point[1] / (point[2] * point[2]);
    T* dummy = nullptr;
    Forward(point, dummy, projected);
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
    bearing[0] = x * inv_norm;
    bearing[1] = y * inv_norm;
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
    const auto lat = atan2(-point[1], sqrt(point[0]*point[0] + point[2]*point[2]));
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
struct Disto24 : CameraFunctor<2, 2, 2>{
  template <class T>
  static void Forward(const T* point, const T* k, T* distorted) {
    const T r2 = SquaredNorm(point);
    const auto distortion = Distortion(r2, k[static_cast<int>(Disto::K1)],
                                       k[static_cast<int>(Disto::K2)]);
    distorted[0] = point[0] * distortion;
    distorted[1] = point[1] * distortion;
  }

  template <class T, bool COMP_PARAM>
  static void ForwardDerivatives(const T* point, const T* k, T* distorted,
                                 T* jacobian) {
    // dx, dy, dk1, dk2
    constexpr int Stride = COMP_PARAM*ParamSize + InSize;
    const auto& k1 = k[static_cast<int>(Disto::K1)];
    const auto& k2 = k[static_cast<int>(Disto::K2)];
    const auto& x = point[0];
    const auto& y = point[1];
    const auto& z = point[2];

    const auto x2 = x * x;
    const auto x4 = x2 * x2;
    const auto y2 = y * y;
    const auto y4 = y2 * y2;
    const auto r2 = x2 + y2;

    jacobian[0] = T(5.0) * k2 * x4 + T(3.0) * k1 * x2 + T(6.0) * k2 * x2 * y2 +
                  k2 * y4 + k1 * y2 + T(1.0);
    jacobian[1] = x * (T(2.0) * k1 * y + T(4.0) * k2 * y * r2);
    jacobian[Stride] = y * (T(2.0) * k1 * x + T(4.0) * k2 * x * r2);
    jacobian[Stride + 1] = T(5.0) * k2 * y4 + T(3.0) * k1 * y2 +
                           T(6.0) * k2 * y2 * x2 + k2 * x4 + k1 * x2 + T(1.0);

    if (COMP_PARAM) {
      jacobian[2] = x * r2;
      jacobian[3] = x * r2 * r2;
      jacobian[Stride + 2] = y * r2;
      jacobian[Stride + 3] = y * r2 * r2;
    }

    Forward(point, k, distorted);
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
struct DistoBrown : CameraFunctor<2, 5, 2>{
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

  template <class T, bool COMP_PARAM>
  static void ForwardDerivatives(const T* point, const T* k, T* distorted,
                                 T* jacobian) {
    // dx, dy, dk1, dk2
    constexpr int Stride = COMP_PARAM*ParamSize + InSize;
    const auto& k1 = k[static_cast<int>(Disto::K1)];
    const auto& k2 = k[static_cast<int>(Disto::K2)];
    const auto& k3 = k[static_cast<int>(Disto::K3)];
    const auto& p1 = k[static_cast<int>(Disto::P1)];
    const auto& p2 = k[static_cast<int>(Disto::P2)];
    const auto& x = point[0];
    const auto& y = point[1];
    const auto& z = point[2];

    const auto x2 = x * x;
    const auto x4 = x2 * x2;
    const auto y2 = y * y;
    const auto y4 = y2 * y2;
    const auto r2 = x2 + y2;

    jacobian[0] = T(7.0) * k3 * x4 * x2 + T(5.0) * k2 * x4 +
                  T(15.0) * k3 * y2 * x4 + T(3.0) * k1 * x2 +
                  T(9.0) * k3 * y4 * x2 + T(6.0) * k2 * y2 * x2 + k3 * y2 * y4 +
                  k2 * y4 + k1 * y2 + T(1.0) + p1 * y + T(6.0) * p2 * x;
    jacobian[1] = x * (T(6.0) * k3 * y * y4 + T(4.0) * k2 * y * y2 +
                       T(12.0) * k3 * x2 * y2 * y + T(2.0) * k1 * y +
                       T(6.0) * k3 * x4 * y + T(4.0) * k2 * x2 * y) +
                  p1 * x + T(2.0) * p2 * y;
    jacobian[Stride] = y * (T(6.0) * k3 * x * x4 + T(4.0) * k2 * x * x2 +
                            T(12.0) * k3 * y2 * x2 * x + T(2.0) * k1 * x +
                            T(6.0) * k3 * y4 * x + T(4.0) * k2 * y2 * x) +
                       p2 * y + T(2.0) * p1 * x;
    jacobian[Stride + 1] =
        T(7.0) * k3 * y4 * y2 + T(5.0) * k2 * y4 + T(15.0) * k3 * x2 * y4 +
        T(3.0) * k1 * y2 + T(9.0) * k3 * x4 * y2 + T(6.0) * k2 * x2 * y2 +
        k3 * x2 * x4 + k2 * x4 + k1 * x2 + T(1.0) + p2 * x + T(6.0) * p1 * y;

    if (COMP_PARAM) {
      jacobian[2] = x * r2;
      jacobian[3] = x * r2 * r2;
      jacobian[4] = x * r2 * r2 * r2;
      jacobian[5] = T(2.0) * x * y;
      jacobian[6] = T(3.0) * x2 + y2;
      jacobian[Stride + 2] = y * r2;
      jacobian[Stride + 3] = y * r2 * r2;
      jacobian[Stride + 4] = y * r2 * r2 * r2;
      jacobian[Stride + 5] = T(3.0) * y2 + x2;
      jacobian[Stride + 6] = T(2.0) * x * y;
    }

    Forward(point, k, distorted);
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
struct Affine : CameraFunctor<2, 4, 2>{
  template <class T>
  static void Forward(const T* point, const T* affine, T* transformed) {
    transformed[0] = affine[0] * point[0] + affine[2];
    transformed[1] = affine[0] * affine[1] * point[1] + affine[3];
  }

  template <class T, bool COMP_PARAM>
  static void ForwardDerivatives(const T* point, const T* affine, T* transformed,
                                 T* jacobian) {
    // dx, dy, dscale
    constexpr int Stride = COMP_PARAM*ParamSize + InSize;
    jacobian[0] = affine[0];
    jacobian[Stride+1] = affine[0] * affine[1];
    jacobian[1] = jacobian[Stride] = T(0.0);
    if(COMP_PARAM){
      jacobian[2] = point[0];
      jacobian[3] = jacobian[5] = T(0.0);
      jacobian[4] = T(1.0);

      jacobian[Stride+2] = point[1]*affine[1];
      jacobian[Stride+3] = point[1]*affine[0];
      jacobian[Stride+4] = T(0.0);
      jacobian[Stride+5] = T(1.0);
    }
    Forward(point, affine, transformed);
  }

  template <class T>
  static void Backward(const T* point, const T* affine, T* transformed) {
    transformed[0] = (point[0] - affine[2])/affine[0];
    transformed[1] = (point[1] - affine[3])/(affine[1]*affine[0]);
  }
};

/* Parameters are : focal */
struct UniformScale : CameraFunctor<2, 1, 2>{
  template <class T>
  static void Forward(const T* point, const T* scale, T* transformed) {
    transformed[0] = scale[0] * point[0];
    transformed[1] = scale[0] * point[1];
  }

  template <class T, bool COMP_PARAM>
  static void ForwardDerivatives(const T* point, const T* scale, T* transformed,
                                 T* jacobian) {
    // dx, dy, dscale
    constexpr int Stride = COMP_PARAM*ParamSize + InSize;
    jacobian[0] = jacobian[Stride+1] = scale[0];
    jacobian[1] = jacobian[Stride] = T(0.0);
    if(COMP_PARAM){
      jacobian[2] = point[0];
      jacobian[Stride+2] = point[1];
    }
    Forward(point, scale, transformed);
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

struct ProjectDerivativesFunction {
  template <class TYPE, class T>
  static void Apply(const T* point, const T* parameters, T* projected, T* jacobian) {
    TYPE::ForwardDerivatives(point, parameters, projected, jacobian);
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
template <class T> struct FunctorTraits { static constexpr int Size = 0;};
template <> struct FunctorTraits<SphericalProjection> { static constexpr int Size = 0;};
template <> struct FunctorTraits<DualProjection> { static constexpr int Size = 1;};
template <> struct FunctorTraits<Disto24> { static constexpr int Size = 2;};
template <> struct FunctorTraits<DistoBrown> {static constexpr int Size = 5;};
template <> struct FunctorTraits<UniformScale> {static constexpr int Size = 1;};
template <> struct FunctorTraits<Affine> { static constexpr int Size = 4;};

template <class PROJ, class DISTO, class AFF>
struct ParametersIndexTraits {
  static constexpr int Projection = 0;    // Always first
  static constexpr int Distorsion = FunctorTraits<PROJ>::Size;
  static constexpr int Affine = FunctorTraits<PROJ>::Size + FunctorTraits<DISTO>::Size;
};

template <class PROJ, class DISTO, class AFF>
struct SizeTraits {
  static constexpr const int ConstexprMax(int a, int b) {
    return (a < b) ? b : a;
  }
  static constexpr int Size =
      ConstexprMax(1, FunctorTraits<PROJ>::Size + FunctorTraits<AFF>::Size +
                          FunctorTraits<DISTO>::Size);
};

// /* Given two function f(x, a) and g(y, b), composed as f o g = f( g(x, b), a)
//  * for which have jacobian stored as df/(dx | da) and dg/(dy | db), apply the
//  * derivative chain-rule in order to get the derivative of
//  *
//  *     (f o g) = d(f o g)/d(x | b | a).
//  */
template <class T, int OutSize1, int Stride1, int ParamSize1, int OutSize2,
          int Stride2, int ParamSize2>
void ComposeDerivatives(
    const Eigen::Matrix<T, OutSize1, Stride1, Eigen::RowMajor>& jacobian1,
    const Eigen::Matrix<T, OutSize2, Stride2, Eigen::RowMajor>& jacobian2,
    T* jacobian) {
  Eigen::Map<Eigen::Matrix<T, OutSize2, Stride1 + ParamSize2, Eigen::RowMajor>>
      jacobian_mapped(jacobian);
  jacobian_mapped.template block<OutSize2, Stride1>(0, 0) =
      jacobian2.template block<OutSize2, OutSize1>(0, 0) * jacobian1;
  jacobian_mapped.template block<OutSize2, ParamSize2>(0, Stride1) =
      jacobian2.template block<OutSize2, ParamSize2>(0, OutSize1);
}

/* Below are some utilities to generalize computation of jacobian of
 * composition of functions f(g(h(i ... ))). Most of the implementation
 * consists in recursing variadic template arguments. Usage is then
 * summarized as : ComposeForwardDerivatives<Func1, Func2, ... FuncN>() */
template <class FUNC>
static constexpr int ComposeStrides() {
  return FUNC::template Stride<true>();
}

template <class FUNC1, class FUNC2, class... FUNCS>
static constexpr int ComposeStrides() {
  return FUNC1::ParamSize + ComposeStrides<FUNC2, FUNCS...>();
}

template <class FUNC>
static constexpr int ComposeIndex() {
  return 0;
}

template <class FUNC1, class FUNC2, class... FUNCS>
static constexpr int ComposeIndex() {
  return FUNC1::ParamSize + ComposeIndex<FUNC2, FUNCS...>();
}

template <class T, class FUNC>
static void ComposeForwardDerivatives(const T* in, const T* parameters, T* out,
                                      T* jacobian) {
  FUNC::template ForwardDerivatives<T, true>(in, parameters, out, jacobian);
}

template <class T, class FUNC1, class FUNC2, class... FUNCS>
static void ComposeForwardDerivatives(const T* in, const T* parameters, T* out,
                                      T* jacobian) {
  constexpr int StrideSub = ComposeStrides<FUNC2, FUNCS...>();
  Eigen::Matrix<T, FUNC2::OutSize, StrideSub, Eigen::RowMajor> sub_jacobian;
  ComposeForwardDerivatives<T, FUNC2, FUNCS...>(in, parameters, out,
                                                sub_jacobian.data());

  constexpr int Index = ComposeIndex<FUNC2, FUNCS...>();
  constexpr int StrideFunc1 = FUNC1::template Stride<true>();
  Eigen::Matrix<T, FUNC1::OutSize, StrideFunc1, Eigen::RowMajor>
      current_jacobian;
  FUNC1::template ForwardDerivatives<T, true>(out, parameters + Index, out,
                                              current_jacobian.data());

  ComposeDerivatives<T, FUNC2::OutSize, StrideSub, FUNC2::ParamSize,
                     FUNC1::OutSize, StrideFunc1, FUNC1::ParamSize>(
      sub_jacobian, current_jacobian, jacobian);
}

template <class T, class FUNC>
static void ComposeForward(const T* in, const T* parameters, T* out) {
  FUNC::template Forward<T>(in, parameters, out);
}

template <class T, class FUNC1, class FUNC2, class... FUNCS>
static void ComposeForward(const T* in, const T* parameters, T* out) {
  T tmp[FUNC2::OutSize];
  ComposeForward<T, FUNC2, FUNCS...>(in, parameters, tmp);

  constexpr int Index = ComposeIndex<FUNC2, FUNCS...>();
  FUNC1::template Forward<T>(&tmp[0], parameters + Index, out);
}

/* Finally, here's the generic camera that implements the PROJ - > DISTO -> AFFINE pattern. */
template <class PROJ, class DISTO, class AFF>
struct ProjectGeneric {
  using Indexes = ParametersIndexTraits<PROJ, DISTO, AFF>;
   static constexpr int Size = SizeTraits<PROJ, DISTO, AFF>::Size;

  template <class T>
  static void Forward(const T* point, const T* parameters, T* projected) {
    ComposeForward<T, AFF, DISTO, PROJ>(point, parameters, projected);
  };

  template <class T>
  static void ForwardDerivatives(const T* point, const T* parameters,
                                 T* projected, T* jacobian) {
    ComposeForwardDerivatives<T, AFF, DISTO, PROJ>(point, parameters, projected,
                                                   jacobian);
  }

  template <class T>
  static void Backward(const T* point, const T* parameters, T* bearing) {
    T tmp[2];
    AFF::Backward(point, parameters + Indexes::Affine, tmp);
    DISTO::Backward(tmp, parameters + Indexes::Distorsion, tmp);
    PROJ::Backward(tmp, parameters + Indexes::Projection, bearing);
  }
};

using PerspectiveCamera = ProjectGeneric<PerspectiveProjection, Disto24, UniformScale>;
using BrownCamera = ProjectGeneric<PerspectiveProjection, DistoBrown, Affine>;
using FisheyeCamera = ProjectGeneric<FisheyeProjection, Disto24, UniformScale>;
// using DualCamera = ProjectGeneric<DualProjection, Disto24, UniformScale>;
// using SphericalCamera = ProjectGeneric<SphericalProjection, Identity, Identity>;

/* This is where the pseudo-strategy pattern takes place. If you want to add
 * your own new camera model, just add a new enum value, the corresponding
 * case below and the implementation (see above). */
template <class FUNC, class... IN>
void Dispatch(const ProjectionType& type, IN&&... args) {
  switch (type) {
    case ProjectionType::PERSPECTIVE:
      FUNC::template Apply<PerspectiveCamera>(std::forward<IN>(args)...);
      break;
    case ProjectionType::BROWN:
      FUNC::template Apply<BrownCamera>(std::forward<IN>(args)...);
      break;
    case ProjectionType::FISHEYE:
      FUNC::template Apply<FisheyeCamera>(std::forward<IN>(args)...);
      break;
    // case ProjectionType::DUAL:
    //   FUNC::template Apply<DualCamera>(std::forward<IN>(args)...);
    //   break;
    // case ProjectionType::SPHERICAL:
    //   FUNC::template Apply<SphericalCamera>(std::forward<IN>(args)...);
      break;
    default:
      throw std::runtime_error("Invalid ProjectionType");
  }
};