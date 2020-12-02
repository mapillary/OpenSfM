#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <features/akaze_bind.h>
#include <features/hahog.h>
#include <features/matching.h>
#include <foundation/python_types.h>

PYBIND11_MODULE(pyfeatures, m) {
  py::enum_<DESCRIPTOR_TYPE>(m, "AkazeDescriptorType")
      .value("SURF_UPRIGHT", SURF_UPRIGHT)
      .value("SURF", SURF)
      .value("MSURF_UPRIGHT", MSURF_UPRIGHT)
      .value("MSURF", MSURF)
      .value("MLDB_UPRIGHT", MLDB_UPRIGHT)
      .value("MLDB", MLDB);

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

  m.def("akaze", features::akaze);

  m.def("hahog", features::hahog, py::arg("image"),
        py::arg("peak_threshold") = 0.003, py::arg("edge_threshold") = 10,
        py::arg("target_num_features") = 0,
        py::arg("use_adaptive_suppression") = false);

  m.def("match_using_words", features::match_using_words);
}
