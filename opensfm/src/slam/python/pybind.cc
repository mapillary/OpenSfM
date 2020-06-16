#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <glog/logging.h>
#include <slam/orb_extractor_bind.h>
#include <slam/guided_matching_bind.h>
#include <slam/slam_utilities.h>
#include <slam/pyslam_utilities.h>
#include <sfm/tracks_manager.h>
namespace py = pybind11;

PYBIND11_MODULE(pyslam, m) {

  py::class_<slam::OrbExtractorWrapper>(m, "OrbExtractor")
    .def(py::init<const unsigned int, const float, const unsigned int,
                  const unsigned int, const unsigned int>(),
        py::arg("max_kpts"),
        py::arg("scale_factor"),
        py::arg("num_levels"),
        py::arg("ini_fast_thr"),
        py::arg("min_fast_thr"))
    .def("extract_to_shot", &slam::OrbExtractorWrapper::extract_to_shot)
    .def("extract", &slam::OrbExtractorWrapper::extract)
    .def("get_scale_levels", &slam::OrbExtractorWrapper::GetScaleLevels)
  ;
  
  py::class_<slam::GuidedMatchingWrapper>(m, "GuidedMatcher")
    .def(py::init<const slam::GridParameters&, const float, const size_t >(),
      py::arg("grid_parameters"), py::arg("scale_factor"), py::arg("num_scale_levels"))
    .def("distribute_undist_keypts_to_grid", &slam::GuidedMatchingWrapper::DistributeUndistKeyptsToGrid,
         py::arg("shot"))
    .def("match_shot_to_shot", &slam::GuidedMatchingWrapper::MatchShotToShot,
         py::arg("shot1"), py::arg("shot2"), py::arg("prev_matched"), py::arg("margin"))
    .def("assign_shot_landmarks_to_kpts_new", &slam::GuidedMatchingWrapper::AssignShot1LandmarksToShot2KptsLM)
    .def("match_for_triangulation_epipolar", &slam::GuidedMatchingWrapper::MatchForTriangulationEpipolar)
    .def("assign_landmarks_to_shot", &slam::GuidedMatchingWrapper::AssignLandmarksToShot)
  ;


  py::class_<slam::GridParameters>(m, "GridParameters")
    .def(py::init<unsigned int, unsigned int, float, float, float, float, float, float>())
  ;
  // Helper class
  py::class_<slam::PySlamUtilities>(m, "SlamUtilities")
    .def("undist_keypts_from_shot", &slam::PySlamUtilities::GetUndistortedKeyptsFromShot)
    .def("keypts_from_shot", &slam::PySlamUtilities::GetKeyptsFromShot)
    .def("compute_descriptor", &slam::PySlamUtilities::SetDescriptorFromObservations)
    .def("compute_normal_and_depth", &slam::PySlamUtilities::SetNormalAndDepthFromObservations)
    .def("compute_min_max_depth", &slam::PySlamUtilities::ComputeMinMaxDepthInShot)
    .def("get_valid_kpts_from_shot", &slam::PySlamUtilities::GetValidKeypts)
    // .def("update_local_keyframes", &slam::PySlamUtilities::UpdateLocalKeyframes, py::return_value_policy::reference_internal)
    // .def("update_local_landmarks", &slam::PySlamUtilities::UpdateLocalLandmarks, py::return_value_policy::reference_internal)
    .def("match_shot_to_local_lms", &slam::PySlamUtilities::MatchShotToLocalLandmarks)
    .def("create_E_21", &slam::PySlamUtilities::create_E_21)
    .def("get_second_order_covisibility_for_shot", &slam::PySlamUtilities::GetSecondOrderCovisibility, py::return_value_policy::reference_internal)
    .def("fuse_duplicated_landmarks", &slam::PySlamUtilities::FuseDuplicatedLandmarks)
    .def("compute_local_keyframes", &slam::PySlamUtilities::ComputeLocalKeyframes, py::return_value_policy::reference_internal)
    // .def("bundle_tracking", &slam::PySlamUtilities::SetUpBAProblem)
    // .def("bundle_single_view", &slam::PySlamUtilities::BundleSingleView)
    .def("triangulate_shot_features", &slam::PySlamUtilities::TriangulateShotFeatures)
    .def("retriangulate", &slam::PySlamUtilities::Retriangulate)
    // .def("get_descriptors", &slam::PySlamUtilities::GetDescriptors)
    // .def("get_keypts_test", &slam::PySlamUtilities::GetKeyptsFromShotTest)
  ;

}
