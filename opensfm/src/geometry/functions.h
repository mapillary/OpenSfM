#pragma once

#include <Eigen/Eigen>

namespace geometry {
template <class T>
inline T SquaredNorm(T* point) {
  return point[0] * point[0] + point[1] * point[1];
}

template <int IN, int P, int OUT>
struct Functor {
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
};  // namespace geometry
