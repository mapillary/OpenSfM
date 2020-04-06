#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <glog/logging.h>

#include <sfm/tracks_manager.h>
#include <sfm/keypoint.h>
#include <foundation/types.h>


PYBIND11_MODULE(pysfm, m) {

  py::class_<Keypoint>(m, "Keypoint")
    .def(py::init<double, double, double, int, int, int, int>())
    .def_readwrite("point", &Keypoint::point)
    .def_readwrite("scale", &Keypoint::scale)
    .def_readwrite("id", &Keypoint::id)
    .def_readwrite("color", &Keypoint::color)
  ;

  py::class_<TracksManager>(m, "TracksManager")
    .def(py::init())
    .def("add_track", &TracksManager::AddTrack)
    .def("get_shot_ids", &TracksManager::GetShotIds)
    .def("get_track_ids", &TracksManager::GetTrackIds)
    .def("get_observation", &TracksManager::GetObservation)
    .def("get_observations_of_point", &TracksManager::GetObservationsOfPoint)
    .def("get_observations_of_point_at_shot", &TracksManager::GetObservationsOfPointsAtShot)
    .def("get_observations_of_shot", &TracksManager::GetObservationsOfShot)
    .def("instanciate_from_file", &TracksManager::InstanciateFromFile)
    .def("write_to_file", &TracksManager::WriteToFile)
    .def("get_all_common_observations", &TracksManager::GetAllCommonObservations)
    .def("get_all_common_observations_all_pairs", &TracksManager::GetAllCommonObservationsAllPairs)
    ;
}

