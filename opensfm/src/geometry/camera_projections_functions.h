#pragma once

#include <foundation/newton_raphson.h>
#include <geometry/functions.h>

namespace geometry {
/* Parameters are : none used */
struct FisheyeProjection : Functor<3, 0, 2> {
  template <class T>
  static void Forward(const T* point, const T* /* p */, T* projected) {
    const T r = sqrt(SquaredNorm(point));
    const auto theta = atan2(r, point[2]);
    projected[0] = theta / r * point[0];
    projected[1] = theta / r * point[1];
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* /* p */, T* projected,
                                 T* jacobian) {
    // dx, dy, dz
    constexpr int stride = Stride<DERIV_PARAMS>();
    const T r2 = SquaredNorm(point);
    const T r = sqrt(r2);
    const T R2 = r2 + point[2] * point[2];
    const T theta = atan2(r, point[2]);
    const T x2 = point[0] * point[0];
    const T y2 = point[1] * point[1];
    const T z2 = point[2] * point[2];

    const T inv_denom = T(1.0) / (r2 * R2 * r);
    jacobian[0] = (x2 * y2 * theta + y2 * y2 * theta + y2 * z2 * theta +
                   x2 * point[2] * r) *
                  inv_denom;
    jacobian[1] = point[0] * (point[1] * point[2] * r - point[1] * theta * R2) *
                  inv_denom;
    jacobian[2] = -point[0] / R2;
    jacobian[stride] = point[1] *
                       (point[0] * point[2] * r - point[0] * theta * R2) *
                       inv_denom;
    jacobian[stride + 1] = (x2 * y2 * theta + x2 * x2 * theta +
                            x2 * z2 * theta + y2 * point[2] * r) *
                           inv_denom;
    jacobian[stride + 2] = -point[1] / R2;

    T* dummy = nullptr;
    Forward(point, dummy, projected);
  }

  template <class T>
  static void Backward(const T* point, const T* /* p */, T* bearing) {
    const T theta = sqrt(SquaredNorm(point));
    T r_div_theta{1.0};
    if (theta > T(1e-8)) {
      const T r = sin(theta);  // r = |(x, y)|
      r_div_theta = r / theta;
    }
    const T x = point[0] * r_div_theta;
    const T y = point[1] * r_div_theta;
    const T z = cos(theta);
    bearing[0] = x;
    bearing[1] = y;
    bearing[2] = z;
  }
};

/* Parameters are : none used */
struct PerspectiveProjection : Functor<3, 0, 2> {
  template <class T>
  static void Forward(const T* point, const T* /* p */, T* projected) {
    projected[0] = point[0] / point[2];
    projected[1] = point[1] / point[2];
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* /* p */, T* projected,
                                 T* jacobian) {
    // dx, dy, dz
    constexpr int stride = Stride<DERIV_PARAMS>();
    jacobian[0] = T(1.0) / point[2];
    jacobian[1] = T(0.0);
    jacobian[2] = -point[0] / (point[2] * point[2]);
    jacobian[stride] = T(0.0);
    jacobian[stride + 1] = jacobian[0];
    jacobian[stride + 2] = -point[1] / (point[2] * point[2]);
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
struct DualProjection : Functor<3, 1, 2> {
  enum { Transition = 0 };

  template <class T>
  static void Forward(const T* point, const T* p, T* projected) {
    T p_persp[2];
    PerspectiveProjection::Forward(point, p, p_persp);
    T p_fish[2];
    FisheyeProjection::Forward(point, p, p_fish);
    projected[0] =
        p[Transition] * p_persp[0] + (1.0 - p[Transition]) * p_fish[0];
    projected[1] =
        p[Transition] * p_persp[1] + (1.0 - p[Transition]) * p_fish[1];
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* p, T* projected,
                                 T* jacobian) {
    // dx, dy, dtransition
    constexpr int stride = Stride<DERIV_PARAMS>();
    T jac_persp[PerspectiveProjection::OutSize *
                PerspectiveProjection::Stride<false>()];
    PerspectiveProjection::ForwardDerivatives<T, false>(point, p, projected,
                                                        &jac_persp[0]);
    T jac_fish[FisheyeProjection::OutSize * FisheyeProjection::Stride<false>()];
    FisheyeProjection::ForwardDerivatives<T, false>(point, p, projected,
                                                    &jac_fish[0]);

    int count = 0;
    for (int i = 0; i < OutSize; ++i) {
      for (int j = 0; j < InSize; ++j) {
        jacobian[i * stride + j] = p[Transition] * jac_persp[count] +
                                   (1.0 - p[Transition]) * jac_fish[count];
        ++count;
      }
    }

    if (DERIV_PARAMS) {
      T p_persp[2];
      PerspectiveProjection::Forward(point, p, p_persp);
      T p_fish[2];
      FisheyeProjection::Forward(point, p, p_fish);
      jacobian[3] = p_persp[0] - p_fish[0];
      jacobian[stride + 3] = p_persp[1] - p_fish[1];
      projected[0] =
          p[Transition] * p_persp[0] + (1.0 - p[Transition]) * p_fish[0];
      projected[1] =
          p[Transition] * p_persp[1] + (1.0 - p[Transition]) * p_fish[1];
    } else {
      Forward(point, p, projected);
    }
  }

  template <class T>
  static void Backward(const T* point, const T* p, T* bearing) {
    // Perform a bit iterations for finding theta from r
    const T r = sqrt(SquaredNorm(point));
    ThetaEval<T> eval_function{0, r, p[Transition]};
    const auto theta_refined =
        foundation::NewtonRaphson<ThetaEval<T>, 1, 1,
                                  foundation::ManualDiff<ThetaEval<T>, 1, 1>>(
            eval_function, 0, iterations);

    const auto s = tan(theta_refined) / (p[Transition] * tan(theta_refined) +
                                         (1.0 - p[Transition]) * theta_refined);

    const T x = s * point[0];
    const T y = s * point[1];
    const T inv_norm = T(1.0) / sqrt(x * x + y * y + T(1.0));
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
struct SphericalProjection : Functor<3, 0, 2> {
  template <class T>
  static void Forward(const T* point, const T* /* p */, T* projected) {
    const T lon = atan2(point[0], point[2]);
    const T lat =
        atan2(-point[1], sqrt(point[0] * point[0] + point[2] * point[2]));
    const T inv_norm = T(1.0 / (2.0 * M_PI));
    projected[0] = lon * inv_norm;
    projected[1] = -lat * inv_norm;
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* /* p */, T* projected,
                                 T* jacobian) {
    // dx, dy
    constexpr int stride = Stride<DERIV_PARAMS>();
    const T rt2 = point[0] * point[0] + point[2] * point[2];
    const T rt = sqrt(rt2);
    const T R2 = SquaredNorm(point) + point[2] * point[2];
    jacobian[0] = point[2] / (T(2.0 * M_PI) * rt2);
    jacobian[1] = T(0.0);
    jacobian[2] = -point[0] / (T(2.0 * M_PI) * rt2);
    jacobian[stride] = -(point[0] * point[1]) / (T(2.0 * M_PI) * R2 * rt);
    jacobian[stride + 1] = rt / (T(2.0 * M_PI) * R2);
    jacobian[stride + 2] = -(point[1] * point[2]) / (T(2.0 * M_PI) * R2 * rt);
    T* dummy = nullptr;
    Forward(point, dummy, projected);
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
}  // namespace geometry
