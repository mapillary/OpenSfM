#include <foundation/newton_raphson.h>

namespace foundation {
template <>
typename TypeTraits<1, 1>::Values SolveDecr<1, 1>(
    const typename TypeTraits<1, 1>::Jacobian& d,
    const typename TypeTraits<1, 1>::Values& f) {
  if (d == 0.) {
    return 0.;
  } else {
    return f / d;
  }
}
}  // namespace foundation
