#include <foundation/python_types.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sfm/observation.h>
#include <sfm/tracks_helpers.h>
#include <sfm/tracks_manager.h>

#include <optional>

PYBIND11_MODULE(pysfm, m) {
  py::class_<Observation>(m, "Observation")
      .def(py::init<double, double, double, int, int, int, int, int, int>(),
           py::arg("x"), py::arg("y"), py::arg("s"), py::arg("r"), py::arg("g"),
           py::arg("b"), py::arg("feature"),
           py::arg("segmentation") = Observation::NO_SEMANTIC_VALUE,
           py::arg("instance") = Observation::NO_SEMANTIC_VALUE)
      .def_readwrite("point", &Observation::point)
      .def_readwrite("scale", &Observation::scale)
      .def_readwrite("id", &Observation::feature_id)
      .def_readwrite("color", &Observation::color)
      .def_readwrite("segmentation", &Observation::segmentation_id)
      .def_readwrite("instance", &Observation::instance_id)
      .def_readonly_static("NO_SEMANTIC_VALUE",
                           &Observation::NO_SEMANTIC_VALUE);

  py::class_<sfm::TracksManager>(m, "TracksManager")
      .def(py::init())
      .def_static("instanciate_from_file",
                  &sfm::TracksManager::InstanciateFromFile,
                  py::call_guard<py::gil_scoped_release>())
      .def_static("instanciate_from_string",
                  &sfm::TracksManager::InstanciateFromString,
                  py::call_guard<py::gil_scoped_release>())
      .def_static("merge_tracks_manager",
                  &sfm::TracksManager::MergeTracksManager)
      .def("add_observation", &sfm::TracksManager::AddObservation)
      .def("remove_observation", &sfm::TracksManager::RemoveObservation)
      .def("num_shots", &sfm::TracksManager::NumShots)
      .def("num_tracks", &sfm::TracksManager::NumTracks)
      .def("get_shot_ids", &sfm::TracksManager::GetShotIds)
      .def("get_track_ids", &sfm::TracksManager::GetTrackIds)
      .def("get_observation", &sfm::TracksManager::GetObservation)
      .def("get_shot_observations", &sfm::TracksManager::GetShotObservations)
      .def("get_track_observations", &sfm::TracksManager::GetTrackObservations)
      .def("construct_sub_tracks_manager",
           &sfm::TracksManager::ConstructSubTracksManager)
      .def("write_to_file", &sfm::TracksManager::WriteToFile)
      .def("as_string", &sfm::TracksManager::AsSring)
      .def("get_all_common_observations",
           &sfm::TracksManager::GetAllCommonObservations,
           py::call_guard<py::gil_scoped_release>())
      .def("get_all_pairs_connectivity",
           &sfm::TracksManager::GetAllPairsConnectivity,
           py::arg("shots") = std::vector<ShotId>(),
           py::arg("tracks") = std::vector<TrackId>(),
           py::call_guard<py::gil_scoped_release>());

  m.def("count_tracks_per_shot", &sfm::tracks_helpers::CountTracksPerShot);
  m.def("add_connections", &sfm::tracks_helpers::AddConnections,
        py::call_guard<py::gil_scoped_release>());
  m.def("remove_connections", &sfm::tracks_helpers::RemoveConnections,
        py::call_guard<py::gil_scoped_release>());
}
