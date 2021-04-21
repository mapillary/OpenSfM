#include <foundation/python_types.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sfm/observation.h>
#include <sfm/sfm_helpers.h>
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

  py::class_<TracksManager>(m, "TracksManager")
      .def(py::init())
      .def_static("instanciate_from_file", &TracksManager::InstanciateFromFile)
      .def_static("instanciate_from_string",
                  &TracksManager::InstanciateFromString)
      .def_static("merge_tracks_manager", &TracksManager::MergeTracksManager)
      .def("add_observation", &TracksManager::AddObservation)
      .def("remove_observation", &TracksManager::RemoveObservation)
      .def("num_shots", &TracksManager::NumShots)
      .def("num_tracks", &TracksManager::NumTracks)
      .def("get_shot_ids", &TracksManager::GetShotIds)
      .def("get_track_ids", &TracksManager::GetTrackIds)
      .def("get_observation", &TracksManager::GetObservation)
      .def("get_shot_observations", &TracksManager::GetShotObservations)
      .def("get_track_observations", &TracksManager::GetTrackObservations)
      .def("construct_sub_tracks_manager",
           &TracksManager::ConstructSubTracksManager)
      .def("write_to_file", &TracksManager::WriteToFile)
      .def("as_string", &TracksManager::AsSring)
      .def("get_all_common_observations",
           &TracksManager::GetAllCommonObservations)
      .def("get_all_pairs_connectivity",
           &TracksManager::GetAllPairsConnectivity,
           py::arg("shots") = std::vector<ShotId>(),
           py::arg("tracks") = std::vector<TrackId>());

  m.def("count_tracks_per_shot", &sfm_helpers::CountTracksPerShot);
}
