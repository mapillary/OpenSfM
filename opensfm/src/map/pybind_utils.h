#pragma once
#include <map/shot.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#ifdef PYBIND11_NAMESPACE_BEGIN
#define PYBIND11_NAMESPACE_BEGIN_ PYBIND11_NAMESPACE_BEGIN
#define PYBIND11_NAMESPACE_END_ PYBIND11_NAMESPACE_END
#else
#define PYBIND11_NAMESPACE_BEGIN_ NAMESPACE_BEGIN
#define PYBIND11_NAMESPACE_END_ NAMESPACE_END
#endif


PYBIND11_NAMESPACE_BEGIN_(PYBIND11_NAMESPACE)
PYBIND11_NAMESPACE_BEGIN_(detail)

// See https://github.com/pybind/pybind11/issues/637
// Also fbsource/fbcode/caffe2/torch/csrc/jit/python/pybind.h

using ListCasterBase =
    pybind11::detail::list_caster<std::vector<map::Landmark *>,
                                  map::Landmark *>;
template <>
struct type_caster<std::vector<map::Landmark *>> : ListCasterBase {
  static handle cast(const std::vector<map::Landmark *> &src,
                     return_value_policy, handle parent) {
    return ListCasterBase::cast(src, return_value_policy::reference, parent);
  }
  static handle cast(const std::vector<map::Landmark *> *src,
                     return_value_policy pol, handle parent) {
    return cast(*src, pol, parent);
  }
};

enum IteratorType {
  // KeyIterator,
  ValueIterator,
  ItemIterator,
  UniquePtrValueIterator,
  UniquePtrIterator,
  RefIterator,
  RefValueIterator
};
template <typename Iterator, typename Sentinel, IteratorType it_type,
          return_value_policy Policy>
struct sfm_iterator_state {
  Iterator it;
  Sentinel end;
  bool first_or_done;
};
PYBIND11_NAMESPACE_END_(detail)

/// Makes an python iterator over the keys (`.first`) of a iterator over pairs
/// from a first and past-the-end InputIterator.
template <return_value_policy Policy = return_value_policy::reference_internal,
          typename Iterator, typename Sentinel,
          typename KeyType = decltype((*std::declval<Iterator>()).second),
          typename... Extra>
iterator make_value_iterator(Iterator first, Sentinel last, Extra &&... extra) {
  typedef detail::sfm_iterator_state<Iterator, Sentinel, detail::RefIterator,
                                     Policy>
      state;

  if (!detail::get_type_info(typeid(state), false)) {
    class_<state>(handle(), "iterator", pybind11::module_local())
        .def("__iter__", [](state &s) -> state & { return s; })
        .def("__next__",
             [](state &s) -> KeyType {
               if (!s.first_or_done)
                 ++s.it;
               else
                 s.first_or_done = false;
               if (s.it == s.end) {
                 s.first_or_done = true;
                 throw stop_iteration();
               }
               return (*s.it).second;
             },
             std::forward<Extra>(extra)..., Policy);
  }

  return cast(state{first, last, true});
}

template <return_value_policy Policy = return_value_policy::reference_internal,
          typename Iterator, typename Sentinel,
          typename KeyType = decltype(&((*std::declval<Iterator>()).second)),
          typename... Extra>
iterator make_ref_value_iterator(Iterator first, Sentinel last,
                                 Extra &&... extra) {
  typedef detail::sfm_iterator_state<Iterator, Sentinel,
                                     detail::RefValueIterator, Policy>
      state;

  if (!detail::get_type_info(typeid(state), false)) {
    class_<state>(handle(), "ref_value_iterator", pybind11::module_local())
        .def("__iter__", [](state &s) -> state & { return s; })
        .def("__next__",
             [](state &s) -> KeyType {
               if (!s.first_or_done)
                 ++s.it;
               else
                 s.first_or_done = false;
               if (s.it == s.end) {
                 s.first_or_done = true;
                 throw stop_iteration();
               }
               return &(s.it->second);
             },
             std::forward<Extra>(extra)..., Policy);
  }

  return cast(state{first, last, true});
}

template <
    return_value_policy Policy = return_value_policy::reference_internal,
    typename Iterator, typename Sentinel,
    typename KeyType =
        pybind11::tuple,  // decltype(&((*std::declval<Iterator>()).second)),
    typename... Extra>
iterator make_ref_iterator(Iterator first, Sentinel last, Extra &&... extra) {
  typedef detail::sfm_iterator_state<Iterator, Sentinel, detail::ValueIterator,
                                     Policy>
      state;

  if (!detail::get_type_info(typeid(state), false)) {
    class_<state>(handle(), "ref_iterator", pybind11::module_local())
        .def("__iter__", [](state &s) -> state & { return s; })
        .def("__next__",
             [](state &s) -> KeyType {
               if (!s.first_or_done)
                 ++s.it;
               else
                 s.first_or_done = false;
               if (s.it == s.end) {
                 s.first_or_done = true;
                 throw stop_iteration();
               }
               return pybind11::make_tuple(s.it->first, &(s.it->second));
             },
             std::forward<Extra>(extra)..., Policy);
  }

  return cast(state{first, last, true});
}

/// Makes a python iterator from a first and past-the-end C++ InputIterator.
template <return_value_policy Policy = return_value_policy::reference_internal,
          typename Iterator, typename Sentinel,
          typename ValueType = decltype(std::declval<Iterator>()),
          typename... Extra>
iterator make_ptr_iterator(Iterator first, Sentinel last, Extra &&... extra) {
  typedef detail::iterator_state<Iterator, Sentinel, false, Policy> state;

  if (!detail::get_type_info(typeid(state), false)) {
    class_<state>(handle(), "iterator", pybind11::module_local())
        .def("__iter__", [](state &s) -> state & { return s; })
        .def("__next__",
             [](state &s) -> ValueType {
               if (!s.first_or_done)
                 ++s.it;
               else
                 s.first_or_done = false;
               if (s.it == s.end) {
                 s.first_or_done = true;
                 throw stop_iteration();
               }
               return s.it;
             },
             std::forward<Extra>(extra)..., Policy);
  }

  return cast(state{first, last, true});
}

/// Makes an iterator over the keys (`.first`) of a stl map-like container
/// supporting `std::begin()`/`std::end()`
template <return_value_policy Policy = return_value_policy::reference_internal,
          typename Type, typename... Extra>
iterator make_value_iterator(Type &value, Extra &&... extra) {
  return make_value_iterator<Policy>(std::begin(value), std::end(value),
                                     extra...);
}
template <return_value_policy Policy = return_value_policy::reference_internal,
          typename Type, typename... Extra>
iterator make_unique_ptr_value_iterator(Type &value, Extra &&... extra) {
  return make_unique_ptr_value_iterator<Policy>(std::begin(value),
                                                std::end(value), extra...);
}
template <return_value_policy Policy = return_value_policy::reference_internal,
          typename Type, typename... Extra>
iterator make_unique_ptr_iterator(Type &value, Extra &&... extra) {
  return make_unique_ptr_iterator<Policy>(std::begin(value), std::end(value),
                                          extra...);
}

template <return_value_policy Policy = return_value_policy::reference_internal,
          typename Type, typename... Extra>
iterator make_ref_value_iterator(Type &value, Extra &&... extra) {
  return make_ref_value_iterator<Policy>(std::begin(value), std::end(value),
                                         extra...);
}
PYBIND11_NAMESPACE_END_(PYBIND11_NAMESPACE)
