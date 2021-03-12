#pragma once

#include <foundation/newton_raphson.h>
#include <foundation/numeric.h>
#include <foundation/types.h>

#include <iostream>
#include <unsupported/Eigen/AutoDiff>

enum class ProjectionType {
  PERSPECTIVE,
  BROWN,
  FISHEYE,
  FISHEYE_OPENCV,
  FISHEYE62,
  SPHERICAL,
  DUAL,
  RADIAL,
  SIMPLE_RADIAL,
  NONE,
};

template <class T>
inline T SquaredNorm(T* point) {
  return point[0] * point[0] + point[1] * point[1];
}

template <int IN, int P, int OUT>
struct CameraFunctor {
  constexpr static int InSize = IN;
  constexpr static int ParamSize = P;
  constexpr static int OutSize = OUT;
  template <bool C>
  static constexpr int Stride() {
    return C * ParamSize + InSize;
  }
};

/* Given two function f(x, a) and g(y, b), composed as f o g = f( g(x, b), a)
 * for which have jacobian stored as df/(dx | da) and dg/(dy | db), apply the
 * derivative chain-rule in order to get the derivative of
 *
 *     (f o g) = d(f o g)/d(x | b | a).
 */
template <class T, bool DERIV_PARAMS, int OutSize1, int Stride1, int ParamSize1,
          int OutSize2, int Stride2, int ParamSize2>
void ComposeDerivatives(
    const Eigen::Matrix<T, OutSize1, Stride1, Eigen::RowMajor>& jacobian1,
    const Eigen::Matrix<T, OutSize2, Stride2, Eigen::RowMajor>& jacobian2,
    T* jacobian) {
  Eigen::Map<Eigen::Matrix<T, OutSize2, Stride1 + DERIV_PARAMS * ParamSize2,
                           Eigen::RowMajor>>
      jacobian_mapped(jacobian);
  jacobian_mapped.template block<OutSize2, Stride1>(0, 0) =
      jacobian2.template block<OutSize2, OutSize1>(0, 0) * jacobian1;
  if (DERIV_PARAMS) {
    jacobian_mapped.template block<OutSize2, ParamSize2>(0, Stride1) =
        jacobian2.template block<OutSize2, ParamSize2>(0, OutSize1);
  }
}

/* Below are some utilities to generalize computation of functions (or their
 * jacobian) of composition of functions f(g(h(i ... ))). Most of the
 * implementation consists in recursing variadic template arguments. Usage is
 * then summarized as : ComposeForwardDerivatives<Func1, Func2, ... FuncN>() */
template <bool DERIV_PARAMS, class FUNC>
static constexpr int ComposeStrides() {
  return FUNC::template Stride<DERIV_PARAMS>();
}

template <bool DERIV_PARAMS, class FUNC1, class FUNC2, class... FUNCS>
static constexpr int ComposeStrides() {
  return DERIV_PARAMS * FUNC1::ParamSize +
         ComposeStrides<DERIV_PARAMS, FUNC2, FUNCS...>();
}

template <class FUNC>
static constexpr int ComposeIndex() {
  return FUNC::ParamSize;
}

template <class FUNC1, class FUNC2, class... FUNCS>
static constexpr int ComposeIndex() {
  return FUNC1::ParamSize + ComposeIndex<FUNC2, FUNCS...>();
}

template <class T, bool DERIV_PARAMS, class FUNC>
static void ComposeForwardDerivatives(const T* in, const T* parameters, T* out,
                                      T* jacobian) {
  FUNC::template ForwardDerivatives<T, DERIV_PARAMS>(in, parameters, out,
                                                     jacobian);
}

template <class T, bool DERIV_PARAMS, class FUNC1, class FUNC2, class... FUNCS>
static void ComposeForwardDerivatives(const T* in, const T* parameters, T* out,
                                      T* jacobian) {
  constexpr int StrideSub = ComposeStrides<DERIV_PARAMS, FUNC2, FUNCS...>();
  Eigen::Matrix<T, FUNC2::OutSize, StrideSub, Eigen::RowMajor> sub_jacobian;
  T tmp[FUNC2::OutSize];
  ComposeForwardDerivatives<T, DERIV_PARAMS, FUNC2, FUNCS...>(
      in, parameters, &tmp[0], sub_jacobian.data());

  constexpr int Index = ComposeIndex<FUNC2, FUNCS...>();
  constexpr int StrideFunc1 = FUNC1::template Stride<DERIV_PARAMS>();
  Eigen::Matrix<T, FUNC1::OutSize, StrideFunc1, Eigen::RowMajor>
      current_jacobian;
  FUNC1::template ForwardDerivatives<T, DERIV_PARAMS>(
      &tmp[0], parameters + Index, out, current_jacobian.data());

  ComposeDerivatives<T, DERIV_PARAMS, FUNC2::OutSize, StrideSub,
                     FUNC2::ParamSize, FUNC1::OutSize, StrideFunc1,
                     FUNC1::ParamSize>(sub_jacobian, current_jacobian,
                                       jacobian);
}

template <class T, class FUNC>
static void ComposeFunctions(const T* in, const T* parameters, T* out) {
  FUNC::template Apply<T>(in, parameters, out);
}

template <class T, class FUNC1, class FUNC2, class... FUNCS>
static void ComposeFunctions(const T* in, const T* parameters, T* out) {
  T tmp[FUNC2::OutSize];
  ComposeFunctions<T, FUNC2, FUNCS...>(in, parameters, tmp);

  constexpr int Index = ComposeIndex<FUNC2, FUNCS...>();
  FUNC1::template Apply<T>(&tmp[0], parameters + Index, out);
}

/* Parameters are : none used */
struct FisheyeProjection : CameraFunctor<3, 0, 2> {
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
struct PerspectiveProjection : CameraFunctor<3, 0, 2> {
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
struct DualProjection : CameraFunctor<3, 1, 2> {
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
struct SphericalProjection : CameraFunctor<3, 0, 2> {
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

/* Parameter is : k1 */
struct Disto2 : CameraFunctor<2, 1, 2> {
  enum { K1 = 0 };

  template <class T>
  static void Forward(const T* point, const T* k, T* distorted) {
    const T r2 = SquaredNorm(point);
    const auto distortion = Distortion(r2, k[static_cast<int>(K1)]);
    distorted[0] = point[0] * distortion;
    distorted[1] = point[1] * distortion;
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* k, T* distorted,
                                 T* jacobian) {
    // dx, dy, dk1
    constexpr int stride = Stride<DERIV_PARAMS>();
    const auto& k1 = k[static_cast<int>(K1)];
    const auto& x = point[0];
    const auto& y = point[1];

    const auto x2 = x * x;
    const auto y2 = y * y;
    const auto r2 = x2 + y2;

    jacobian[0] = T(3.0) * k1 * x2 + k1 * y2 + T(1.0);
    jacobian[1] = x * T(2.0) * k1 * y;
    jacobian[stride] = y * T(2.0) * k1 * x;
    jacobian[stride + 1] = k1 * (T(3.0) * y2 + x2) + T(1.0);

    if (DERIV_PARAMS) {
      jacobian[2] = x * r2;
      jacobian[stride + 2] = y * r2;
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
    DistoEval<T> eval_function{rd, k[static_cast<int>(K1)]};
    const auto ru_refined =
        foundation::NewtonRaphson<DistoEval<T>, 1, 1,
                                  foundation::ManualDiff<DistoEval<T>, 1, 1>>(
            eval_function, rd, iterations);

    // Compute distortion factor from undistorted radius
    const T r2 = ru_refined * ru_refined;
    const auto distortion = Distortion(r2, k[static_cast<int>(K1)]);

    // Unapply undistortion
    undistorted[0] = point[0] / distortion;
    undistorted[1] = point[1] / distortion;
  }

  static constexpr int iterations = 10;
  template <class T>
  struct DistoEval {
    const T& rd;
    const T& k1;
    T operator()(const T& x) const {
      const auto r2 = x * x;
      return x * Disto2::Distortion(r2, k1) - rd;
    }
    T derivative(const T& x) const {
      const auto r2 = x * x;
      return Disto2::DistortionDerivative(r2, k1);
    }
  };

  template <class T>
  static inline T Distortion(const T& r2, const T& k1) {
    return T(1.0) + r2 * k1;
  }

  template <class T>
  static T DistortionDerivative(const T& r2, const T& k1) {
    return T(1.0) + r2 * T(2.0) * k1;
  }
};

/* Parameters are : k1, k2 */
struct Disto24 : CameraFunctor<2, 2, 2> {
  enum { K1 = 0, K2 = 1 };

  template <class T>
  static void Forward(const T* point, const T* k, T* distorted) {
    const T r2 = SquaredNorm(point);
    const auto distortion =
        Distortion(r2, k[static_cast<int>(K1)], k[static_cast<int>(K2)]);
    distorted[0] = point[0] * distortion;
    distorted[1] = point[1] * distortion;
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* k, T* distorted,
                                 T* jacobian) {
    // dx, dy, dk1, dk2
    constexpr int stride = Stride<DERIV_PARAMS>();
    const auto& k1 = k[static_cast<int>(K1)];
    const auto& k2 = k[static_cast<int>(K2)];
    const auto& x = point[0];
    const auto& y = point[1];

    const auto x2 = x * x;
    const auto x4 = x2 * x2;
    const auto y2 = y * y;
    const auto y4 = y2 * y2;
    const auto r2 = x2 + y2;

    jacobian[0] = T(5.0) * k2 * x4 + T(3.0) * k1 * x2 + T(6.0) * k2 * x2 * y2 +
                  k2 * y4 + k1 * y2 + T(1.0);
    jacobian[1] = x * (T(2.0) * k1 * y + T(4.0) * k2 * y * r2);
    jacobian[stride] = y * (T(2.0) * k1 * x + T(4.0) * k2 * x * r2);
    jacobian[stride + 1] = T(5.0) * k2 * y4 + T(3.0) * k1 * y2 +
                           T(6.0) * k2 * y2 * x2 + k2 * x4 + k1 * x2 + T(1.0);

    if (DERIV_PARAMS) {
      jacobian[2] = x * r2;
      jacobian[3] = x * r2 * r2;
      jacobian[stride + 2] = y * r2;
      jacobian[stride + 3] = y * r2 * r2;
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
    DistoEval<T> eval_function{rd, k[static_cast<int>(K1)],
                               k[static_cast<int>(K2)]};
    const auto ru_refined =
        foundation::NewtonRaphson<DistoEval<T>, 1, 1,
                                  foundation::ManualDiff<DistoEval<T>, 1, 1>>(
            eval_function, rd, iterations);

    // Compute distortion factor from undistorted radius
    const T r2 = ru_refined * ru_refined;
    const auto distortion =
        Distortion(r2, k[static_cast<int>(K1)], k[static_cast<int>(K2)]);

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

/* Parameters are : k1, k2, k3, k4 */
struct Disto2468 : CameraFunctor<2, 4, 2> {
  enum { K1 = 0, K2 = 1, K3 = 2, K4 = 3 };

  template <class T>
  static void Forward(const T* point, const T* k, T* distorted) {
    const T r2 = SquaredNorm(point);
    const auto distortion =
        Distortion(r2, k[static_cast<int>(K1)], k[static_cast<int>(K2)],
                   k[static_cast<int>(K3)], k[static_cast<int>(K4)]);
    distorted[0] = point[0] * distortion;
    distorted[1] = point[1] * distortion;
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* k, T* distorted,
                                 T* jacobian) {
    // dx, dy, dk1, dk2, dk3, dk4
    constexpr int stride = Stride<DERIV_PARAMS>();
    const auto& k1 = k[static_cast<int>(K1)];
    const auto& k2 = k[static_cast<int>(K2)];
    const auto& k3 = k[static_cast<int>(K3)];
    const auto& k4 = k[static_cast<int>(K4)];
    const auto& x = point[0];
    const auto& y = point[1];

    const auto x2 = x * x;
    const auto x4 = x2 * x2;
    const auto y2 = y * y;
    const auto y4 = y2 * y2;
    const auto r2 = x2 + y2;

    jacobian[0] =
        x * (T(2.0) * k1 * x + T(4.0) * k2 * x * r2 +
             T(6.0) * k3 * x * r2 * r2 + T(8.0) * k4 * x * r2 * r2 * r2) +
        k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2 + k4 * r2 * r2 * r2 * r2 +
        T(1.0);
    jacobian[1] =
        x * (T(2.0) * k1 * y + T(4.0) * k2 * y * r2 +
             T(6.0) * k3 * y * r2 * r2 + T(8.0) * k4 * y * r2 * r2 * r2);
    jacobian[stride] =
        y * (T(2.0) * k1 * x + T(4.0) * k2 * x * r2 +
             T(6.0) * k3 * x * r2 * r2 + T(8.0) * k4 * x * r2 * r2 * r2);
    jacobian[stride + 1] =
        y * (T(2.0) * k1 * y + T(4.0) * k2 * y * r2 +
             T(6.0) * k3 * y * r2 * r2 + T(8.0) * k4 * y * r2 * r2 * r2) +
        k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2 + k4 * r2 * r2 * r2 * r2 +
        T(1.0);

    if (DERIV_PARAMS) {
      jacobian[2] = x * r2;
      jacobian[3] = x * r2 * r2;
      jacobian[4] = x * r2 * r2 * r2;
      jacobian[5] = x * r2 * r2 * r2 * r2;
      jacobian[stride + 2] = y * r2;
      jacobian[stride + 3] = y * r2 * r2;
      jacobian[stride + 4] = y * r2 * r2 * r2;
      jacobian[stride + 5] = y * r2 * r2 * r2 * r2;
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
    DistoEval<T> eval_function{rd, k[static_cast<int>(K1)],
                               k[static_cast<int>(K2)], k[static_cast<int>(K3)],
                               k[static_cast<int>(K4)]};
    const auto ru_refined =
        foundation::NewtonRaphson<DistoEval<T>, 1, 1,
                                  foundation::ManualDiff<DistoEval<T>, 1, 1>>(
            eval_function, rd, iterations);

    // Compute distortion factor from undistorted radius
    const T r2 = ru_refined * ru_refined;
    const auto distortion =
        Distortion(r2, k[static_cast<int>(K1)], k[static_cast<int>(K2)],
                   k[static_cast<int>(K3)], k[static_cast<int>(K4)]);

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
    const T& k3;
    const T& k4;
    T operator()(const T& x) const {
      const auto r2 = x * x;
      return x * Disto2468::Distortion(r2, k1, k2, k3, k4) - rd;
    }
    T derivative(const T& x) const {
      const auto r2 = x * x;
      return Disto2468::DistortionDerivative(r2, k1, k2, k3, k4);
    }
  };

  template <class T>
  static inline T Distortion(const T& r2, const T& k1, const T& k2, const T& k3,
                             const T& k4) {
    return T(1.0) + r2 * (k1 + r2 * (k2 + r2 * (k3 + r2 * k4)));
  }

  template <class T>
  static T DistortionDerivative(const T& r2, const T& k1, const T& k2,
                                const T& k3, const T& k4) {
    return T(1.0) +
           r2 * (T(3.0) * k1 +
                 r2 * (T(5.0) * k2 + r2 * (T(7.0) * k3 + r2 * T(9.0) * k4)));
  }
};

/* Parameters are : k1, k2, k3, k4, k5, k6, p1, p2 */
struct Disto62 : CameraFunctor<2, 8, 2> {
  enum { K1 = 0, K2 = 1, K3 = 2, K4 = 3, K5 = 4, K6 = 5, P1 = 6, P2 = 7 };

  template <class T>
  static void Forward(const T* point, const T* k, T* distorted) {
    const auto r2 = SquaredNorm(point);
    // Radial
    const auto distortion_radial =
        RadialDistortion(r2, k[static_cast<int>(K1)], k[static_cast<int>(K2)],
                         k[static_cast<int>(K3)], k[static_cast<int>(K4)],
                         k[static_cast<int>(K5)], k[static_cast<int>(K6)]);

    // Tangential
    const auto distortion_tangential =
        TangentialDistortion(r2, point[0], point[1], k[static_cast<int>(P1)],
                             k[static_cast<int>(P2)]);
    distorted[0] = point[0] * distortion_radial + distortion_tangential[0];
    distorted[1] = point[1] * distortion_radial + distortion_tangential[1];
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* k, T* distorted,
                                 T* jacobian) {
    constexpr int stride = Stride<DERIV_PARAMS>();
    const auto& k1 = k[static_cast<int>(K1)];
    const auto& k2 = k[static_cast<int>(K2)];
    const auto& k3 = k[static_cast<int>(K3)];
    const auto& k4 = k[static_cast<int>(K4)];
    const auto& k5 = k[static_cast<int>(K5)];
    const auto& k6 = k[static_cast<int>(K6)];
    const auto& p1 = k[static_cast<int>(P1)];
    const auto& p2 = k[static_cast<int>(P2)];
    const auto& x = point[0];
    const auto& y = point[1];

    const auto x2 = x * x;
    const auto y2 = y * y;
    const auto r2 = x2 + y2;
    const auto r2_2 = r2 * r2;
    const auto r2_3 = r2_2 * r2;
    const auto r2_4 = r2_3 * r2;
    const auto r2_5 = r2_4 * r2;

    // Compute tangential distortion separatedly
    const auto dx_dxt = T(2.0) * y * p1 + T(6.0) * p2 * x;
    const auto dx_dyt = T(2.0) * x * p1 + T(2.0) * p2 * y;
    const auto dy_dxt = dx_dyt;  // == dx_dyt
    const auto dy_dyt = T(2.0) * x * p2 + T(6.0) * p1 * y;
    // Computing the deriv. of the rad dist. p(x,y):
    // For x: d x * p(x,y)/ dx = p(x,y) + x * dp(x,y)/dy
    // and simplify dp(x,y)/dx = dp/dr * dr/dx
    const auto p = RadialDistortion(r2, k1, k2, k3, k4, k5, k6);
    const auto dr_dx = T(2.0) * x;  // dr/dx = (x^2 + y^2)' = 2x
    const auto dr_dy = T(2.0) * y;  // dr/dy = (x^2 + y^2)' = 2y
    const auto dp_dr = k1 + T(2.0) * k2 * r2 + T(3.0) * k3 * r2_2 +
                       T(4.0) * k4 * r2_3 + T(5.0) * k5 * r2_4 +
                       T(6.0) * k6 * r2_5;
    jacobian[0] = p + x * dp_dr * dr_dx + dx_dxt;
    jacobian[1] = x * dp_dr * dr_dy + dx_dyt;
    jacobian[stride + 0] = y * dp_dr * dr_dx + dy_dxt;
    jacobian[stride + 1] = p + y * dp_dr * dr_dy + dy_dyt;

    if (DERIV_PARAMS) {
      const auto r2_6 = r2_5 * r2;
      // K1 - K6
      jacobian[2] = x * r2;
      jacobian[3] = x * r2_2;
      jacobian[4] = x * r2_3;
      jacobian[5] = x * r2_4;
      jacobian[6] = x * r2_5;
      jacobian[7] = x * r2_6;
      // P1 - P2
      jacobian[8] = T(2.0) * x * y;
      jacobian[9] = T(3.0) * x2 + y2;
      // K1 - K6
      jacobian[stride + 2] = y * r2;
      jacobian[stride + 3] = y * r2_2;
      jacobian[stride + 4] = y * r2_3;
      jacobian[stride + 5] = y * r2_4;
      jacobian[stride + 6] = y * r2_5;
      jacobian[stride + 7] = y * r2_6;
      // P1 - P2
      jacobian[stride + 8] = T(3.0) * y2 + x2;
      jacobian[stride + 9] = jacobian[8];
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
    // Compute undistorted radius
    DistoEval<T> eval_function{mapped_point,
                               k[static_cast<int>(K1)],
                               k[static_cast<int>(K2)],
                               k[static_cast<int>(K3)],
                               k[static_cast<int>(K4)],
                               k[static_cast<int>(K5)],
                               k[static_cast<int>(K6)],
                               k[static_cast<int>(P1)],
                               k[static_cast<int>(P2)]};
    Eigen::Map<Vec2<T>> mapped_undistorted(undistorted);
    mapped_undistorted =
        foundation::NewtonRaphson<DistoEval<T>, 2, 2,
                                  foundation::ManualDiff<DistoEval<T>, 2, 2>>(
            eval_function, mapped_point, iterations);
  }

  static constexpr int iterations = 10;
  template <class T>
  struct DistoEval {
    const Vec2<T>& point_distorted;
    const T& k1;
    const T& k2;
    const T& k3;
    const T& k4;
    const T& k5;
    const T& k6;
    const T& p1;
    const T& p2;
    Vec2<T> operator()(const Vec2<T>& point) const {
      const T r2 = point.squaredNorm();
      const auto distortion_radial =
          RadialDistortion(r2, k1, k2, k3, k4, k5, k6);

      const auto distortion_tangential =
          TangentialDistortion(r2, point[0], point[1], p1, p2);
      return point * distortion_radial + distortion_tangential -
             point_distorted;
    }

    Mat2<T> derivative(const Vec2<T>& point) const {
      Mat2<T> jacobian;
      std::array<double, 2> dummy;
      std::array<double, 8> ks{k1, k2, k3, k4, k5, k6, p1, p2};
      Disto62::ForwardDerivatives<T, false>(point.data(), ks.data(),
                                            dummy.data(), jacobian.data());
      return jacobian;
    }
  };

  template <class T>
  static T RadialDistortion(const T& r2, const T& k1, const T& k2, const T& k3,
                            const T& k4, const T& k5, const T& k6) {
    return T(1.0) +
           r2 * (k1 + r2 * (k2 + r2 * (k3 + r2 * (k4 + r2 * (k5 + r2 * k6)))));
  }

  template <class T>
  static Vec2<T> TangentialDistortion(const T& r2, const T& x, const T& y,
                                      const T& p1, const T& p2) {
    return Vec2<T>(T(2.0) * p1 * x * y + p2 * (r2 + T(2.0) * x * x),
                   T(2.0) * p2 * x * y + p1 * (r2 + T(2.0) * y * y));
  }
};

/* Parameters are : k1, k2, k3, p1, p2 */
struct DistoBrown : CameraFunctor<2, 5, 2> {
  enum { K1 = 0, K2 = 1, K3 = 2, P1 = 3, P2 = 4 };

  template <class T>
  static void Forward(const T* point, const T* k, T* distorted) {
    const auto r2 = SquaredNorm(point);
    const auto distortion_radial =
        RadialDistortion(r2, k[static_cast<int>(K1)], k[static_cast<int>(K2)],
                         k[static_cast<int>(K3)]);
    const auto distortion_tangential =
        TangentialDistortion(r2, point[0], point[1], k[static_cast<int>(P1)],
                             k[static_cast<int>(P2)]);
    distorted[0] = point[0] * distortion_radial + distortion_tangential[0];
    distorted[1] = point[1] * distortion_radial + distortion_tangential[1];
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* k, T* distorted,
                                 T* jacobian) {
    // dx, dy, dk1, dk2, dk3, dp1, dp2
    constexpr int stride = Stride<DERIV_PARAMS>();
    const auto& k1 = k[static_cast<int>(K1)];
    const auto& k2 = k[static_cast<int>(K2)];
    const auto& k3 = k[static_cast<int>(K3)];
    const auto& p1 = k[static_cast<int>(P1)];
    const auto& p2 = k[static_cast<int>(P2)];
    const auto& x = point[0];
    const auto& y = point[1];

    const T x2 = x * x;
    const T x4 = x2 * x2;
    const T y2 = y * y;
    const T y4 = y2 * y2;
    const T r2 = x2 + y2;
    const T r4 = r2 * r2;
    const T r6 = r4 * r2;

    jacobian[0] = T(5.0) * k2 * x4 + T(3.0) * k1 * x2 + T(6.0) * k3 * x2 * r4 +
                  T(6.0) * k2 * x2 * y2 + k3 * r6 + k2 * y4 + k1 * y2 + T(1.0) +
                  T(2.0) * p1 * y + T(6.0) * p2 * x;
    jacobian[1] =
        x * (T(2.0) * k1 * y + T(4.0) * k2 * y * r2 + T(6.0) * k3 * y * r4) +
        T(2.0) * p1 * x + T(2.0) * p2 * y;
    jacobian[stride + 1] = T(5.0) * k2 * y4 + T(3.0) * k1 * y2 +
                           T(6.0) * k3 * y2 * r4 + T(6.0) * k2 * x2 * y2 +
                           k3 * r6 + k2 * x4 + k1 * x2 + T(1.0) +
                           T(2.0) * p2 * x + T(6.0) * p1 * y;
    jacobian[stride] =
        y * (T(2.0) * k1 * x + T(4.0) * k2 * x * r2 + T(6.0) * k3 * x * r4) +
        T(2.0) * p2 * y + T(2.0) * p1 * x;

    if (DERIV_PARAMS) {
      jacobian[2] = x * r2;
      jacobian[3] = x * r2 * r2;
      jacobian[4] = x * r2 * r2 * r2;
      jacobian[5] = T(2.0) * x * y;
      jacobian[6] = T(3.0) * x2 + y2;
      jacobian[stride + 2] = y * r2;
      jacobian[stride + 3] = y * r2 * r2;
      jacobian[stride + 4] = y * r2 * r2 * r2;
      jacobian[stride + 5] = T(3.0) * y2 + x2;
      jacobian[stride + 6] = T(2.0) * x * y;
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
                               k[static_cast<int>(K1)],
                               k[static_cast<int>(K2)],
                               k[static_cast<int>(K3)],
                               k[static_cast<int>(P1)],
                               k[static_cast<int>(P2)]};
    Eigen::Map<Vec2<T>> mapped_undistorted(undistorted);
    mapped_undistorted =
        foundation::NewtonRaphson<DistoEval<T>, 2, 2,
                                  foundation::ManualDiff<DistoEval<T>, 2, 2>>(
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
      Mat2<T> jacobian;
      std::array<double, 2> dummy;
      std::array<double, 5> ks{k1, k2, k3, p1, p2};
      DistoBrown::ForwardDerivatives<T, false>(point.data(), ks.data(),
                                               dummy.data(), jacobian.data());
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
struct Affine : CameraFunctor<2, 4, 2> {
  enum { Focal = 0, AspectRatio = 1, Cx = 2, Cy = 3 };

  template <class T>
  static void Forward(const T* point, const T* affine, T* transformed) {
    transformed[0] = affine[Focal] * point[0] + affine[Cx];
    transformed[1] =
        affine[Focal] * affine[AspectRatio] * point[1] + affine[Cy];
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* affine,
                                 T* transformed, T* jacobian) {
    // dx, dy, dfocal, daspect_ratio, dcx, dcy
    constexpr int stride = Stride<DERIV_PARAMS>();
    jacobian[0] = affine[Focal];
    jacobian[stride + 1] = affine[Focal] * affine[AspectRatio];
    jacobian[1] = jacobian[stride] = T(0.0);
    if (DERIV_PARAMS) {
      jacobian[2] = point[0];
      jacobian[3] = jacobian[5] = T(0.0);
      jacobian[4] = T(1.0);

      jacobian[stride + 2] = point[1] * affine[AspectRatio];
      jacobian[stride + 3] = point[1] * affine[Focal];
      jacobian[stride + 4] = T(0.0);
      jacobian[stride + 5] = T(1.0);
    }
    Forward(point, affine, transformed);
  }

  template <class T>
  static void Backward(const T* point, const T* affine, T* transformed) {
    transformed[0] = (point[0] - affine[Cx]) / affine[Focal];
    transformed[1] =
        (point[1] - affine[Cy]) / (affine[AspectRatio] * affine[Focal]);
  }
};

/* Parameters are : focal */
struct UniformScale : CameraFunctor<2, 1, 2> {
  enum { Focal = 0 };

  template <class T>
  static void Forward(const T* point, const T* scale, T* transformed) {
    transformed[0] = scale[Focal] * point[0];
    transformed[1] = scale[Focal] * point[1];
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* scale, T* transformed,
                                 T* jacobian) {
    // dx, dy, dscale
    constexpr int stride = Stride<DERIV_PARAMS>();
    jacobian[0] = jacobian[stride + 1] = scale[Focal];
    jacobian[1] = jacobian[stride] = T(0.0);
    if (DERIV_PARAMS) {
      jacobian[2] = point[0];
      jacobian[stride + 2] = point[1];
    }
    Forward(point, scale, transformed);
  }

  template <class T>
  static void Backward(const T* point, const T* scale, T* transformed) {
    transformed[0] = point[0] / scale[Focal];
    transformed[1] = point[1] / scale[Focal];
  }
};

/* Parameters are : none used */
struct Identity : CameraFunctor<2, 0, 2> {
  template <class T>
  static void Forward(const T* point, const T* /* k */, T* transformed) {
    transformed[0] = point[0];
    transformed[1] = point[1];
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* /* k */,
                                 T* transformed, T* jacobian) {
    // dx, dy
    constexpr int stride = Stride<DERIV_PARAMS>();
    jacobian[0] = jacobian[stride + 1] = T(1.0);
    jacobian[1] = jacobian[stride] = T(0.0);
    T* dummy = nullptr;
    Forward(point, dummy, transformed);
  }

  template <class T>
  static void Backward(const T* point, const T* /* k */, T* transformed) {
    transformed[0] = point[0];
    transformed[1] = point[1];
  }
};

struct Pose : CameraFunctor<3, 6, 3> {
  /* Rotation and translation being stored as angle-axis | translation, apply
   the transformation : x_world = R(t)*(x_camera - translation) by directly
   applying the angle-axis rotation */
  enum { Rx = 0, Ry = 1, Rz = 2, Tx = 3, Ty = 4, Tz = 5 };
  template <class T>
  static void Forward(const T* point, const T* rt, T* transformed) {
    const T x = point[0] - rt[Tx];
    const T y = point[1] - rt[Ty];
    const T z = point[2] - rt[Tz];

    const T a = -rt[Rx];
    const T b = -rt[Ry];
    const T c = -rt[Rz];

    // Dot product of angle-axis and the point
    const T cp_x = b * z - c * y;
    const T cp_y = c * x - a * z;
    const T cp_z = a * y - b * x;

    const T theta2 = a * a + b * b + c * c;
    // Regular angle-axis transformation
    if (theta2 > T(std::numeric_limits<double>::epsilon())) {
      const T theta = sqrt(theta2);
      const T inv_theta = T(1.0) / theta;
      const T cos_theta = cos(theta);
      const T sin_theta = sin(theta) / theta;
      const T dot_pt_p =
          (a * x + b * y + c * z) * (T(1.0) - cos_theta) / theta2;
      transformed[0] = x * cos_theta + sin_theta * cp_x + a * dot_pt_p;
      transformed[1] = y * cos_theta + sin_theta * cp_y + b * dot_pt_p;
      transformed[2] = z * cos_theta + sin_theta * cp_z + c * dot_pt_p;
      // Apply taylor approximation for small angles
    } else {
      transformed[0] = x + cp_x;
      transformed[1] = y + cp_y;
      transformed[2] = z + cp_z;
    }
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* rt, T* transformed,
                                 T* jacobian) {
    // dx, dy, dz, drx, dry, drz, dtz, dty, dtz
    if (DERIV_PARAMS) {
      using Dual = Eigen::AutoDiffScalar<Vec3d>;

      /* Get jacobian or R wrt. angle-axis using Dual */
      Dual r_diff[InSize];
      r_diff[0].value() = -rt[Rx];
      r_diff[0].derivatives() = -Vec3d::Unit(Rx);
      r_diff[1].value() = -rt[Ry];
      r_diff[1].derivatives() = -Vec3d::Unit(Ry);
      r_diff[2].value() = -rt[Rz];
      r_diff[2].derivatives() = -Vec3d::Unit(Rz);

      Eigen::Matrix<Dual, 3, 3, Eigen::RowMajor> rotation;
      AngleAxisToRotation(&r_diff[0], rotation.data());

      /* Storage is row-ordered : R00, R01, R02, R10, ... R22 */
      Eigen::Matrix<T, 9, 3, Eigen::RowMajor> rotation_angleaxis;
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          for (int k = 0; k < 3; ++k) {
            rotation_angleaxis(i * 3 + j, k) = rotation(i, j).derivatives()(k);
          }
        }
      }

      /* R.(x-t) derivatives are pretty straightfoward : dR00, dR01, ... dR22,
       * dx, dy, dz, dtx, dty, dtz */
      const T xyz[] = {point[0] - rt[Tx], point[1] - rt[Ty], point[2] - rt[Tz]};
      Eigen::Matrix<T, 3, 15, Eigen::RowMajor> point_rotation =
          Eigen::Matrix<T, 3, 15, Eigen::RowMajor>::Zero();
      for (int i = 0; i < 3; ++i) {
        // dRij
        for (int j = 0; j < 3; ++j) {
          point_rotation(i, i * 3 + j) = xyz[j];
        }
        // dx, dy, dz
        for (int j = 0; j < 3; ++j) {
          point_rotation(i, 9 + j) = rotation(i, j).value();
        }
        // dtx, dty, dtz
        for (int j = 0; j < 3; ++j) {
          point_rotation(i, 12 + j) = -point_rotation(i, 9 + j);
        }
      }

      /* Compose d(R) / d(angle axis) with d(pose) / d(R | t | x) in order to
       * get d(pose) / d(angle axis | t | x) */

      ComposeDerivatives<T, true, 9, 3, 0, 3, 15, 6>(rotation_angleaxis,
                                                     point_rotation, jacobian);

      /* Re-orde from angle-axis | x | t to x | angle-axis | t */
      for (int i = 0; i < 3; ++i) {
        // Swap dai and dxi
        for (int j = 0; j < 3; ++j) {
          std::swap(jacobian[i * 9 + j], jacobian[i * 9 + 3 + j]);
        }
      }
    } else {
      double minus_r[] = {-rt[Rx], -rt[Ry], -rt[Rz]};
      AngleAxisToRotation(&minus_r[0], jacobian);
    }
    Forward(point, rt, transformed);
  }

 private:
  template <class T>
  static void AngleAxisToRotation(const T* angle_axis, T* rotation) {
    const T theta2 = SquaredNorm(angle_axis) + angle_axis[2] * angle_axis[2];

    // Use Taylor approximation near zero angle : R = I + [angle_axis]x
    if (theta2 < T(std::numeric_limits<double>::epsilon())) {
      Eigen::Map<Mat3<T>> mapped_rotation(rotation);
      const Eigen::Map<const Vec3<T>> mapped_angle_axis(angle_axis);
      foundation::SkewMatrixT(mapped_angle_axis, &mapped_rotation);
      for (int i = 0; i < 3; ++i) {
        rotation[i * 3 + i] = T(1.0);
      }
      // From
      // https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/
    } else {
      const T theta = sqrt(theta2);
      const T c = cos(theta);
      const T s = sin(theta);
      const T t = T(1.0) - c;

      const T inv_theta2 = T(1.0) / theta2;
      const T inv_theta = T(1.0) / theta;

      const T xx = angle_axis[0] * angle_axis[0] * inv_theta2;
      const T xy = angle_axis[0] * angle_axis[1] * inv_theta2;
      const T xz = angle_axis[0] * angle_axis[2] * inv_theta2;
      const T yy = angle_axis[1] * angle_axis[1] * inv_theta2;
      const T yz = angle_axis[1] * angle_axis[2] * inv_theta2;
      const T zz = angle_axis[2] * angle_axis[2] * inv_theta2;

      const T xs = angle_axis[0] * inv_theta * s;
      const T ys = angle_axis[1] * inv_theta * s;
      const T zs = angle_axis[2] * inv_theta * s;

      rotation[0] = t * xx + c;
      rotation[1] = t * xy - zs;
      rotation[2] = t * xz + ys;

      rotation[3] = t * xy + zs;
      rotation[4] = t * yy + c;
      rotation[5] = t * yz - xs;

      rotation[6] = t * xz - ys;
      rotation[7] = t * yz + xs;
      rotation[8] = t * zz + c;
    }
  }
};

struct Normalize : CameraFunctor<3, 0, 3> {
  template <class T>
  static void Forward(const T* point, const T* /* k */, T* transformed) {
    const T inv_norm = T(1.0) / sqrt(SquaredNorm(point) + point[2] * point[2]);
    for (int i = 0; i < 3; ++i) {
      transformed[i] = point[i] * inv_norm;
    }
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* /* k */,
                                 T* transformed, T* jacobian) {
    // dx, dy
    constexpr int stride = Stride<DERIV_PARAMS>();

    const T& x = point[0];
    const T& y = point[1];
    const T& z = point[2];
    const T x2 = x * x;
    const T y2 = y * y;
    const T z2 = z * z;
    const T norm2 = x2 + y2 + z2;
    const T norm = sqrt(norm2);
    const T inv_norm32 = T(1.0) / (norm * norm2);

    jacobian[0] = (y2 + z2) * inv_norm32;
    jacobian[1] = (-x * y) * inv_norm32;
    jacobian[2] = (-x * z) * inv_norm32;

    jacobian[stride] = (-y * x) * inv_norm32;
    jacobian[stride + 1] = (x2 + z2) * inv_norm32;
    jacobian[stride + 2] = (-y * z) * inv_norm32;

    jacobian[2 * stride] = (-z * x) * inv_norm32;
    jacobian[2 * stride + 1] = (-z * y) * inv_norm32;
    jacobian[2 * stride + 2] = (x2 + y2) * inv_norm32;

    T* dummy = nullptr;
    Forward(point, dummy, transformed);
  }
};

template <class FUNC>
struct ForwardWrapper : public FUNC {
  template <class T>
  static void Apply(const T* in, const T* parameters, T* out) {
    FUNC::Forward(in, parameters, out);
  }
};

template <class FUNC>
struct BackwardWrapper : public FUNC {
  template <class T>
  static void Apply(const T* in, const T* parameters, T* out) {
    FUNC::Backward(in, parameters, out);
  }
};

struct ProjectPoseDerivatives {
  template <class TYPE, class T>
  static void Apply(const T* point, const T* parameters, T* projected,
                    T* jacobian) {
    ComposeForwardDerivatives<T, true, TYPE, Pose>(point, parameters, projected,
                                                   jacobian);
  }
};

struct ProjectPosePointDerivatives {
  template <class TYPE, class T>
  static void Apply(const T* point, const T* parameters, T* projected,
                    T* jacobian) {
    ComposeForwardDerivatives<T, false, TYPE, Pose>(point, parameters,
                                                    projected, jacobian);
  }
};

struct ProjectRigPoseDerivatives {
  template <class TYPE, class T>
  static void Apply(const T* point, const T* parameters, T* projected,
                    T* jacobian) {
    ComposeForwardDerivatives<T, true, TYPE, Pose, Pose>(point, parameters,
                                                         projected, jacobian);
  }
};

struct PoseNormalizedDerivatives {
  template <class TYPE, class T>
  static void Apply(const T* point, const T* parameters, T* projected,
                    T* jacobian) {
    ComposeForwardDerivatives<T, true, Normalize, Pose>(point, parameters,
                                                        projected, jacobian);
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
  static void Apply(const T* point, const T* parameters, T* projected,
                    T* jacobian) {
    TYPE::template ForwardDerivatives<T, true>(point, parameters, projected,
                                               jacobian);
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
template <class T>
struct FunctorTraits {
  static constexpr int Size = 0;
};
template <>
struct FunctorTraits<SphericalProjection> {
  static constexpr int Size = 0;
};
template <>
struct FunctorTraits<DualProjection> {
  static constexpr int Size = 1;
};
template <>
struct FunctorTraits<Disto2> {
  static constexpr int Size = 1;
};
template <>
struct FunctorTraits<Disto24> {
  static constexpr int Size = 2;
};
template <>
struct FunctorTraits<Disto2468> {
  static constexpr int Size = 4;
};
template <>
struct FunctorTraits<Disto62> {
  static constexpr int Size = 8;
};
template <>
struct FunctorTraits<DistoBrown> {
  static constexpr int Size = 5;
};
template <>
struct FunctorTraits<UniformScale> {
  static constexpr int Size = 1;
};
template <>
struct FunctorTraits<Affine> {
  static constexpr int Size = 4;
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

/* Finally, here's the generic camera that implements the PROJ - > DISTO ->
 * AFFINE pattern. */
template <class PROJ, class DISTO, class AFF>
struct ProjectGeneric
    : CameraFunctor<3, SizeTraits<PROJ, DISTO, AFF>::Size, 2> {
  using ProjectionType = PROJ;
  using DistoType = DISTO;
  using AffineType = AFF;

  static constexpr int Size = SizeTraits<PROJ, DISTO, AFF>::Size;

  template <class T>
  static void Forward(const T* point, const T* parameters, T* projected) {
    ComposeFunctions<T, ForwardWrapper<AFF>, ForwardWrapper<DISTO>,
                     ForwardWrapper<PROJ>>(point, parameters, projected);
  };

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* parameters,
                                 T* projected, T* jacobian) {
    ComposeForwardDerivatives<T, DERIV_PARAMS, AFF, DISTO, PROJ>(
        point, parameters, projected, jacobian);
  }

  template <class T>
  static void Backward(const T* point, const T* parameters, T* bearing) {
    T parameters_backward[Size];
    ConstructReversedParams(parameters, parameters_backward);
    ComposeFunctions<T, BackwardWrapper<PROJ>, BackwardWrapper<DISTO>,
                     BackwardWrapper<AFF>>(point, parameters_backward, bearing);
  }

 private:
  template <class T>
  static void ConstructReversedParams(const T* parameters_forward,
                                      T* parameters_backward) {
    int count = 0;
    int index = Size - FunctorTraits<PROJ>::Size;
    for (int i = 0; i < FunctorTraits<PROJ>::Size; ++i) {
      parameters_backward[index + i] = parameters_forward[count++];
    }
    index -= FunctorTraits<DISTO>::Size;
    for (int i = 0; i < FunctorTraits<DISTO>::Size; ++i) {
      parameters_backward[index + i] = parameters_forward[count++];
    }
    index -= FunctorTraits<AFF>::Size;
    for (int i = 0; i < FunctorTraits<AFF>::Size; ++i) {
      parameters_backward[index + i] = parameters_forward[count++];
    }
  }
};

using PerspectiveCamera =
    ProjectGeneric<PerspectiveProjection, Disto24, UniformScale>;
using RadialCamera = ProjectGeneric<PerspectiveProjection, Disto24, Affine>;
using SimpleRadialCamera =
    ProjectGeneric<PerspectiveProjection, Disto2, Affine>;
using BrownCamera = ProjectGeneric<PerspectiveProjection, DistoBrown, Affine>;
using FisheyeCamera = ProjectGeneric<FisheyeProjection, Disto24, UniformScale>;
using FisheyeOpencvCamera =
    ProjectGeneric<FisheyeProjection, Disto2468, Affine>;
using Fisheye62Camera = ProjectGeneric<FisheyeProjection, Disto62, Affine>;
using DualCamera = ProjectGeneric<DualProjection, Disto24, UniformScale>;
using SphericalCamera = ProjectGeneric<SphericalProjection, Identity, Identity>;

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
    case ProjectionType::FISHEYE_OPENCV:
      FUNC::template Apply<FisheyeOpencvCamera>(std::forward<IN>(args)...);
      break;
    case ProjectionType::FISHEYE62:
      FUNC::template Apply<Fisheye62Camera>(std::forward<IN>(args)...);
      break;
    case ProjectionType::RADIAL:
      FUNC::template Apply<RadialCamera>(std::forward<IN>(args)...);
      break;
    case ProjectionType::SIMPLE_RADIAL:
      FUNC::template Apply<SimpleRadialCamera>(std::forward<IN>(args)...);
      break;
    case ProjectionType::DUAL:
      FUNC::template Apply<DualCamera>(std::forward<IN>(args)...);
      break;
    case ProjectionType::SPHERICAL:
      FUNC::template Apply<SphericalCamera>(std::forward<IN>(args)...);
      break;
    case ProjectionType::NONE:
    default:
      throw std::runtime_error("Invalid ProjectionType");
  }
};
