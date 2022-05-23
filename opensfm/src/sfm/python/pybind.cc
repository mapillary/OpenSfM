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
  py::module::import("opensfm.pymap");
  py::module::import("opensfm.pygeometry");
  py::module::import("opensfm.pybundle");

  m.def("count_tracks_per_shot", &sfm::tracks_helpers::CountTracksPerShot);
  m.def("add_connections", &sfm::tracks_helpers::AddConnections,
        py::call_guard<py::gil_scoped_release>());
  m.def("remove_connections", &sfm::tracks_helpers::RemoveConnections,
        py::call_guard<py::gil_scoped_release>());

  py::class_<sfm::BAHelpers>(m, "BAHelpers")
      .def_static("bundle", &sfm::BAHelpers::Bundle)
      .def_static("bundle_local", &sfm::BAHelpers::BundleLocal)
      .def_static("bundle_shot_poses", &sfm::BAHelpers::BundleShotPoses)
      .def_static("bundle_to_map", &sfm::BAHelpers::BundleToMap)
      .def_static("shot_neighborhood_ids", &sfm::BAHelpers::ShotNeighborhoodIds)
      .def_static("detect_alignment_constraints",
                  &sfm::BAHelpers::DetectAlignmentConstraints)
      .def_static("add_gcp_to_bundle", &sfm::BAHelpers::AddGCPToBundle);

  m.def("realign_maps", &sfm::retriangulation::RealignMaps,
        py::call_guard<py::gil_scoped_release>());
}
