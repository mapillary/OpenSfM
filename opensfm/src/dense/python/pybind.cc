#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <glog/logging.h>

#include "depthmap_bind.h"
#include "openmvs_exporter.h"


namespace py = pybind11;


PYBIND11_MODULE(csfm, m) {
  google::InitGoogleLogging("csfm");

  py::class_<csfm::OpenMVSExporter>(m, "OpenMVSExporter")
    .def(py::init())
    .def("add_camera", &csfm::OpenMVSExporter::AddCamera)
    .def("add_shot", &csfm::OpenMVSExporter::AddShot)
    .def("add_point", &csfm::OpenMVSExporter::AddPoint)
    .def("export", &csfm::OpenMVSExporter::Export)
  ;

  py::class_<csfm::DepthmapEstimatorWrapper>(m, "DepthmapEstimator")
    .def(py::init())
    .def("set_depth_range", &csfm::DepthmapEstimatorWrapper::SetDepthRange)
    .def("set_patchmatch_iterations", &csfm::DepthmapEstimatorWrapper::SetPatchMatchIterations)
    .def("set_patch_size", &csfm::DepthmapEstimatorWrapper::SetPatchSize)
    .def("set_min_patch_sd", &csfm::DepthmapEstimatorWrapper::SetMinPatchSD)
    .def("add_view", &csfm::DepthmapEstimatorWrapper::AddView)
    .def("compute_patch_match", &csfm::DepthmapEstimatorWrapper::ComputePatchMatch)
    .def("compute_patch_match_sample", &csfm::DepthmapEstimatorWrapper::ComputePatchMatchSample)
    .def("compute_brute_force", &csfm::DepthmapEstimatorWrapper::ComputeBruteForce)
  ;

  py::class_<csfm::DepthmapCleanerWrapper>(m, "DepthmapCleaner")
    .def(py::init())
    .def("set_same_depth_threshold", &csfm::DepthmapCleanerWrapper::SetSameDepthThreshold)
    .def("set_min_consistent_views", &csfm::DepthmapCleanerWrapper::SetMinConsistentViews)
    .def("add_view", &csfm::DepthmapCleanerWrapper::AddView)
    .def("clean", &csfm::DepthmapCleanerWrapper::Clean)
  ;

  py::class_<csfm::DepthmapPrunerWrapper>(m, "DepthmapPruner")
    .def(py::init())
    .def("set_same_depth_threshold", &csfm::DepthmapPrunerWrapper::SetSameDepthThreshold)
    .def("add_view", &csfm::DepthmapPrunerWrapper::AddView)
    .def("prune", &csfm::DepthmapPrunerWrapper::Prune)
  ;
}
