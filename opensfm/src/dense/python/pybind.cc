#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <dense/depthmap_bind.h>
#include <dense/openmvs_exporter.h>
#include <foundation/python_types.h>

PYBIND11_MODULE(pydense, m) {
  py::class_<dense::OpenMVSExporter>(m, "OpenMVSExporter")
      .def(py::init())
      .def("add_camera", &dense::OpenMVSExporter::AddCamera)
      .def("add_shot", &dense::OpenMVSExporter::AddShot)
      .def("add_point", &dense::OpenMVSExporter::AddPoint)
      .def("export", &dense::OpenMVSExporter::Export);

  py::class_<dense::DepthmapEstimatorWrapper>(m, "DepthmapEstimator")
      .def(py::init())
      .def("set_depth_range", &dense::DepthmapEstimatorWrapper::SetDepthRange)
      .def("set_patchmatch_iterations",
           &dense::DepthmapEstimatorWrapper::SetPatchMatchIterations)
      .def("set_patch_size", &dense::DepthmapEstimatorWrapper::SetPatchSize)
      .def("set_min_patch_sd", &dense::DepthmapEstimatorWrapper::SetMinPatchSD)
      .def("add_view", &dense::DepthmapEstimatorWrapper::AddView)
      .def("compute_patch_match",
           &dense::DepthmapEstimatorWrapper::ComputePatchMatch)
      .def("compute_patch_match_sample",
           &dense::DepthmapEstimatorWrapper::ComputePatchMatchSample)
      .def("compute_brute_force",
           &dense::DepthmapEstimatorWrapper::ComputeBruteForce);

  py::class_<dense::DepthmapCleanerWrapper>(m, "DepthmapCleaner")
      .def(py::init())
      .def("set_same_depth_threshold",
           &dense::DepthmapCleanerWrapper::SetSameDepthThreshold)
      .def("set_min_consistent_views",
           &dense::DepthmapCleanerWrapper::SetMinConsistentViews)
      .def("add_view", &dense::DepthmapCleanerWrapper::AddView)
      .def("clean", &dense::DepthmapCleanerWrapper::Clean);

  py::class_<dense::DepthmapPrunerWrapper>(m, "DepthmapPruner")
      .def(py::init())
      .def("set_same_depth_threshold",
           &dense::DepthmapPrunerWrapper::SetSameDepthThreshold)
      .def("add_view", &dense::DepthmapPrunerWrapper::AddView)
      .def("prune", &dense::DepthmapPrunerWrapper::Prune);
}
