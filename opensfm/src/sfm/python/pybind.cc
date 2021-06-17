#include <foundation/python_types.h>
#include <map/observation.h>
#include <map/tracks_manager.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sfm/ba_helpers.h>
#include <sfm/retriangulation.h>
#include <sfm/tracks_helpers.h>

#include <optional>

PYBIND11_MODULE(pysfm, m) {
  m.def("count_tracks_per_shot", &sfm::tracks_helpers::CountTracksPerShot);
  m.def("add_connections", &sfm::tracks_helpers::AddConnections,
        py::call_guard<py::gil_scoped_release>());
  m.def("remove_connections", &sfm::tracks_helpers::RemoveConnections,
        py::call_guard<py::gil_scoped_release>());

  py::class_<sfm::BAHelpers>(m, "BAHelpers")
      .def("bundle", &sfm::BAHelpers::Bundle)
      .def("bundle_local", &sfm::BAHelpers::BundleLocal)
      .def("bundle_shot_poses", &sfm::BAHelpers::BundleShotPoses)
      .def("shot_neighborhood_ids", &sfm::BAHelpers::ShotNeighborhoodIds)
      .def("detect_alignment_constraints",
           &sfm::BAHelpers::DetectAlignmentConstraints);

  m.def("realign_points", &sfm::retriangulation::RealignPoints,
        py::call_guard<py::gil_scoped_release>());
}
