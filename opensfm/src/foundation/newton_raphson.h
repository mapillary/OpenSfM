#pragma once

#include <limits>

template< class F>
struct FiniteDiff{
  static double Derivative(const F& func, double x){
    const auto eps = std::numeric_limits<double>::epsilon();
    return (func(x+eps)-func(x))/eps;
  }
};

template< class F>
struct ManualDiff{
  static double Derivative(const F& func, double x){
    return func.derivative(x);
  }
};

template< class F, class D = FiniteDiff<F> >
double NewtonRaphson(const F& func, double initial_value, int iterations, double tol=2e-4){
  const auto eps = std::numeric_limits<double>::epsilon();
  auto current_value = initial_value;
  for (int i = 0; i < iterations; ++i){
    const auto at_current_value = func(current_value);
    const auto derivative = D::Derivative(func, current_value);
    const auto decr = at_current_value/derivative;
    if(std::fabs(decr) < tol){
      break;
    }
    current_value -= decr;
  }
  return current_value;
}
