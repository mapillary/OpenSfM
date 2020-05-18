#include <foundation/newton_raphson.h>

template <>
typename TypeTraits<1, 1>::Values SolveDecr<1, 1>(
    const typename TypeTraits<1, 1>::Jacobian& d,
    const typename TypeTraits<1, 1>::Values& f) {
  return f / d;
}