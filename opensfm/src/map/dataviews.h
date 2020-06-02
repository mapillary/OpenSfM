#include <pybind11/pybind11.h>
// #include <pybind11/common.h>
#include <map/map.h>
NAMESPACE_BEGIN(PYBIND11_NAMESPACE)

/// Makes an python iterator over the keys (`.first`) of a iterator over pairs from a
/// first and past-the-end InputIterator.
template <return_value_policy Policy = return_value_policy::reference_internal,
          typename Iterator,
          typename Sentinel,
          typename KeyType = decltype((*std::declval<Iterator>()).second),
          typename... Extra>
iterator make_value_iterator(Iterator first, Sentinel last, Extra &&... extra) {
    typedef detail::iterator_state<Iterator, Sentinel, true, Policy> state;

    if (!detail::get_type_info(typeid(state), false)) {
        class_<state>(handle(), "iterator", pybind11::module_local())
            .def("__iter__", [](state &s) -> state& { return s; })
            .def("__next__", [](state &s) -> KeyType {
                if (!s.first_or_done)
                    ++s.it;
                else
                    s.first_or_done = false;
                if (s.it == s.end) {
                    s.first_or_done = true;
                    throw stop_iteration();
                }
                return (*s.it).second;
            }, std::forward<Extra>(extra)..., Policy);
    }

    return cast(state{first, last, true});
}
template <return_value_policy Policy = return_value_policy::reference_internal,
          typename Iterator,
          typename Sentinel,
        //   typename KeyType = decltype((*std::declval<Iterator>()).second),
          typename ValueType = std::pair<decltype((*std::declval<Iterator>()).first), 
          std::pointer_traits<(*std::declval<Iterator>()).second>::element_type>,
        //   std::pointer_traits<(decltype((*std::declval<Iterator>()).second)>::element_type*>,
          typename... Extra>
iterator make_unique_ptr_iterator(Iterator first, Sentinel last, Extra &&... extra) {
    typedef detail::iterator_state<Iterator, Sentinel, true, Policy> state;

    if (!detail::get_type_info(typeid(state), false)) {
        class_<state>(handle(), "iterator", pybind11::module_local())
            .def("__iter__", [](state &s) -> state& { return s; })
            .def("__next__", [](state &s) -> KeyType {
                if (!s.first_or_done)
                    ++s.it;
                else
                    s.first_or_done = false;
                if (s.it == s.end) {
                    s.first_or_done = true;
                    throw stop_iteration();
                }
                return (*s.it).second;
            }, std::forward<Extra>(extra)..., Policy);
    }

    return cast(state{first, last, true});
}

template <return_value_policy Policy = return_value_policy::reference_internal,
          typename Iterator,
          typename Sentinel,
          typename KeyType = decltype((*std::declval<Iterator>()).second),
          typename... Extra>
iterator make_unique_ptr_value_iterator(Iterator first, Sentinel last, Extra &&... extra) {
    typedef detail::iterator_state<Iterator, Sentinel, true, Policy> state;

    if (!detail::get_type_info(typeid(state), false)) {
        class_<state>(handle(), "iterator", pybind11::module_local())
            .def("__iter__", [](state &s) -> state& { return s; })
            .def("__next__", [](state &s) -> KeyType {
                if (!s.first_or_done)
                    ++s.it;
                else
                    s.first_or_done = false;
                if (s.it == s.end) {
                    s.first_or_done = true;
                    throw stop_iteration();
                }
                return (*s.it).second.get();
            }, std::forward<Extra>(extra)..., Policy);
    }

    return cast(state{first, last, true});
}
/// Makes an iterator over the keys (`.first`) of a stl map-like container supporting
/// `std::begin()`/`std::end()`
template <return_value_policy Policy = return_value_policy::reference_internal,
          typename Type, typename... Extra> iterator make_value_iterator(Type &value, Extra&&... extra) {
    return make_value_iterator<Policy>(std::begin(value), std::end(value), extra...);
}
template <return_value_policy Policy = return_value_policy::reference_internal,
          typename Type, typename... Extra> iterator make_unique_ptr_value_iterator(Type &value, Extra&&... extra) {
    return make_unique_ptr_value_iterator<Policy>(std::begin(value), std::end(value), extra...);
}
template <return_value_policy Policy = return_value_policy::reference_internal,
          typename Type, typename... Extra> iterator make_unique_ptr_iterator(Type &value, Extra&&... extra) {
    return make_unique_ptr_iterator<Policy>(std::begin(value), std::end(value), extra...);
}
NAMESPACE_END(PYBIND11_NAMESPACE)

namespace map
{
class ShotView
{
public:
  ShotView(Map& map):map_(map){
      const auto& shots = map_.GetAllShots();
      auto it = shots.begin();
      std::unordered_map<ShotId, Shot*>::iterator t()
  }
// private:
  Map& map_;
};

class PointView
{
public:
  PointView(Map& map):map_(map){}
private:
  Map& map_;
};

class CameraView
{
public:
  CameraView(Map& map):map_(map){}
private:
  Map& map_;
};

}
