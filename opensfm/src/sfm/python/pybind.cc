#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <glog/logging.h>

#include <sfm/tracks_manager.h>
#include <foundation/types.h>


PYBIND11_MODULE(pysfm, m) {
  py::class_<TracksManager>(m, "TracksManager")
    .def(py::init())
    .def("get_shot_ids", &TracksManager::GetShotIds)
    .def("get_track_ids", &TracksManager::GetTrackIds)
    .def("get_observation", &TracksManager::GetObservation)
    .def("get_observations_of_point", &TracksManager::GetObservationsOfPoint)
    .def("get_observations_of_point_at_shot", &TracksManager::GetObservationsOfPointsAtShot)
    .def("get_observations_of_shot", &TracksManager::GetObservationsOfShot)
    .def("instanciate_from_file", &TracksManager::InstanciateFromFile)
    .def("get_all_common_observations", &TracksManager::GetAllCommonObservations)
    ;
}

