#pragma once

#include <foundation/newton_raphson.h>
#include <foundation/types.h>
#include <geometry/functions.h>

#include <array>

namespace geometry {
/* Parameter is : k1 */
struct Disto2 : Functor<2, 1, 2> {
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
struct Disto24 : Functor<2, 2, 2> {
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
struct Disto2468 : Functor<2, 4, 2> {
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
    const auto y2 = y * y;
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
struct Disto62 : Functor<2, 8, 2> {
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
struct DistoBrown : Functor<2, 5, 2> {
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
}  // namespace geometry
