#include <features/akaze_bind.h>
#include <features/hahog.h>
#include <features/matching.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <stdexcept>
#include <string>
#include <utility>

namespace {

// Funnel every C++ exception leaving a binding into a type pybind11 is
// guaranteed to translate.
//
// Without this, an exception pybind cannot translate -- or one escaping a
// noexcept frame -- reaches std::terminate and kills the interpreter outright.
// The caller then dies with no Python unwinding, so it records no failure and
// its work is silently lost rather than retried (T234633115: this is the top
// remaining source of stranded Mapillary clusters, ~224 crashes/day, 94% of
// them std::terminate, at 12-44 GB RSS). A Python exception instead lets the
// caller log the failure, shrink its batch, and retry.
//
// Args are deduced from the wrapped function, so reference parameters stay
// references and nothing is copied that was not copied before.
template <typename R, typename... Args>
auto guarded(const char* name, R (*fn)(Args...)) {
  return [name, fn](Args... args) -> R {
    try {
      return fn(std::forward<Args>(args)...);
    } catch (const std::exception& e) {
      throw std::runtime_error(std::string(name) + ": " + e.what());
    } catch (...) {
      throw std::runtime_error(std::string(name) + ": unknown C++ exception");
    }
  };
}

}  // namespace

PYBIND11_MODULE(pyfeatures, m) {
  py::enum_<DESCRIPTOR_TYPE>(m, "AkazeDescriptorType")
      .value("SURF_UPRIGHT", SURF_UPRIGHT)
      .value("SURF", SURF)
      .value("MSURF_UPRIGHT", MSURF_UPRIGHT)
      .value("MSURF", MSURF)
      .value("MLDB_UPRIGHT", MLDB_UPRIGHT)
      .value("MLDB", MLDB);
  py::enum_<DIFFUSIVITY_TYPE>(m, "AkazeDiffusivityType")
      .value("PM_G1", PM_G1)
      .value("PM_G2", PM_G2)
      .value("WEICKERT", WEICKERT)
      .value("CHARBONNIER", CHARBONNIER);

  py::class_<AKAZEOptions>(m, "AKAZEOptions")
      .def(py::init())
      .def_readwrite("omin", &AKAZEOptions::omin)
      .def_readwrite("omax", &AKAZEOptions::omax)
      .def_readwrite("nsublevels", &AKAZEOptions::nsublevels)
      .def_readwrite("img_width", &AKAZEOptions::img_width)
      .def_readwrite("img_height", &AKAZEOptions::img_height)
      .def_readwrite("soffset", &AKAZEOptions::soffset)
      .def_readwrite("derivative_factor", &AKAZEOptions::derivative_factor)
      .def_readwrite("sderivatives", &AKAZEOptions::sderivatives)
      .def_readwrite("diffusivity", &AKAZEOptions::diffusivity)
      .def_readwrite("dthreshold", &AKAZEOptions::dthreshold)
      .def_readwrite("min_dthreshold", &AKAZEOptions::min_dthreshold)
      .def_readwrite("target_num_features", &AKAZEOptions::target_num_features)
      .def_readwrite("use_adaptive_suppression",
                     &AKAZEOptions::use_adaptive_suppression)
      .def_readwrite("descriptor", &AKAZEOptions::descriptor)
      .def_readwrite("descriptor_size", &AKAZEOptions::descriptor_size)
      .def_readwrite("descriptor_channels", &AKAZEOptions::descriptor_channels)
      .def_readwrite("descriptor_pattern_size",
                     &AKAZEOptions::descriptor_pattern_size)
      .def_readwrite("kcontrast", &AKAZEOptions::kcontrast)
      .def_readwrite("kcontrast_percentile",
                     &AKAZEOptions::kcontrast_percentile)
      .def_readwrite("kcontrast_nbins", &AKAZEOptions::kcontrast_nbins)
      .def_readwrite("use_isotropic_diffusion",
                     &AKAZEOptions::use_isotropic_diffusion)
      .def_readwrite("save_scale_space", &AKAZEOptions::save_scale_space)
      .def_readwrite("save_keypoints", &AKAZEOptions::save_keypoints)
      .def_readwrite("verbosity", &AKAZEOptions::verbosity);

  m.def("akaze", guarded("akaze", features::akaze));

  m.def("hahog", guarded("hahog", features::hahog), py::arg("image"),
        py::arg("peak_threshold") = 0.003, py::arg("edge_threshold") = 10,
        py::arg("target_num_features") = 0);

  m.def("match_using_words",
        guarded("match_using_words", features::match_using_words));
  // The guard is inside the released-GIL region, so the exception is already a
  // translatable type by the time gil_scoped_release's destructor re-acquires
  // the GIL during unwinding.
  m.def("compute_vlad_descriptor",
        guarded("compute_vlad_descriptor", features::compute_vlad_descriptor),
        py::call_guard<py::gil_scoped_release>());
  m.def("compute_vlad_distances",
        guarded("compute_vlad_distances", features::compute_vlad_distances),
        py::call_guard<py::gil_scoped_release>());
}
