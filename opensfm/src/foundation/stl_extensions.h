#pragma once

#include <type_traits>

namespace foundation {
template <class T>
constexpr std::add_const_t<T>& as_const(T& t) noexcept {
  return t;
}
}  // namespace foundation
