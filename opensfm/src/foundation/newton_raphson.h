#pragma once

#include <Eigen/Eigen>
#include <limits>

namespace foundation {

// Unfortunately we need these traits because we want to use
// straight double fo the scalar (N=1, M=1) case. Otherwise,
// wrapping in an Eigen object is killing the performance
template <int N, int M>
struct TypeTraits {
  using Jacobian = Eigen::Matrix<double, N, M>;
  using Values = Eigen::Matrix<double, N, 1>;
  static double Norm(const Values& x) { return x.norm(); }
};

template <>
struct TypeTraits<1, 1> {
  using Jacobian = double;
  using Values = double;
  static double Norm(const Values& x) { return std::fabs(x); }
};

// Class for computing the jacobian using finite differencing.
// Very slow and prone to non-convergence.
template <class F, int N, int M>
struct FiniteDiff {
  static typename TypeTraits<N, M>::Jacobian Derivative(
      const F& func, typename TypeTraits<N, M>::Values& x) {
    typename TypeTraits<N, M>::Jacobian jacobian;
    typename TypeTraits<N, M>::Values x_plus = x;
    constexpr auto eps = 1e-15;
    for (int i = 0; i < M; ++i) {
      x_plus[i] += eps;
      jacobian.col(i) = (func(x_plus) - func(x)) / eps;
      x_plus[i] -= eps;
    }
    return jacobian;
  }
};

template <class F>
struct FiniteDiff<F, 1, 1> {
  static typename TypeTraits<1, 1>::Jacobian Derivative(
      const F& func, typename TypeTraits<1, 1>::Values& x) {
    typename TypeTraits<1, 1>::Jacobian jacobian;
    constexpr auto eps = 1e-15;
    return (func(x + eps) - func(x)) / eps;
  }
};

// Class when the client know how to compute the jacobian.
template <class F, int N, int M>
struct ManualDiff {
  static typename TypeTraits<N, M>::Jacobian Derivative(
      const F& func, const typename TypeTraits<N, M>::Values& x) {
    return func.derivative(x);
  }
};

// Again, we need to specialize that one for the scalar case.
template <int N, int M>
typename TypeTraits<N, M>::Values SolveDecr(
    const typename TypeTraits<N, M>::Jacobian& d,
    const typename TypeTraits<N, M>::Values& f) {
  return (d.transpose() * d).inverse() * d.transpose() * f;
};

template <>
typename TypeTraits<1, 1>::Values SolveDecr<1, 1>(
    const typename TypeTraits<1, 1>::Jacobian& d,
    const typename TypeTraits<1, 1>::Values& f);

// Performs Newton-Raphson iterations for root finding of a function.
template <class F, int N, int M, class D = FiniteDiff<F, N, M>>
typename TypeTraits<N, M>::Values NewtonRaphson(
    const F& func, const typename TypeTraits<N, M>::Values& initial_value,
    int iterations, double tol = 1e-6) {
  constexpr auto eps = std::numeric_limits<double>::epsilon();
  auto current_value = initial_value;
  for (int i = 0; i < iterations; ++i) {
    const auto at_current_value = func(current_value);
    const auto derivative = D::Derivative(func, current_value);
    const auto decr = SolveDecr<N, M>(derivative, at_current_value);
    if (TypeTraits<N, M>::Norm(decr) < tol) {
      break;
    }
    current_value -= decr;
  }
  return current_value;
}
}  // namespace foundation
