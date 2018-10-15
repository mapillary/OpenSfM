#include <iostream>
#include <vector>
#include <pybind11/pybind11.h>

#include "types.h"
#include "hahog.cc"
#include "multiview.cc"
#include "akaze.cc"
#include "bundle.h"
#include "openmvs_exporter.h"
#include "depthmap_wrapper.cc"
#include "reconstruction_alignment.h"


namespace py = pybind11;


PYBIND11_MODULE(csfm, m) {
  google::InitGoogleLogging("csfm");

  py::enum_<DESCRIPTOR_TYPE>(m, "AkazeDescriptorType")
    .value("SURF_UPRIGHT", SURF_UPRIGHT)
    .value("SURF", SURF)
    .value("MSURF_UPRIGHT", MSURF_UPRIGHT)
    .value("MSURF", MSURF)
    .value("MLDB_UPRIGHT", MLDB_UPRIGHT)
    .value("MLDB", MLDB)
  ;

  py::class_<AKAZEOptions>(m, "AKAZEOptions")
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
    .def_readwrite("use_adaptive_suppression", &AKAZEOptions::use_adaptive_suppression)
    .def_readwrite("descriptor", &AKAZEOptions::descriptor)
    .def_readwrite("descriptor_size", &AKAZEOptions::descriptor_size)
    .def_readwrite("descriptor_channels", &AKAZEOptions::descriptor_channels)
    .def_readwrite("descriptor_pattern_size", &AKAZEOptions::descriptor_pattern_size)
    .def_readwrite("kcontrast", &AKAZEOptions::kcontrast)
    .def_readwrite("kcontrast_percentile", &AKAZEOptions::kcontrast_percentile)
    .def_readwrite("kcontrast_nbins", &AKAZEOptions::kcontrast_nbins)
    .def_readwrite("use_isotropic_diffusion", &AKAZEOptions::use_isotropic_diffusion)
    .def_readwrite("save_scale_space", &AKAZEOptions::save_scale_space)
    .def_readwrite("save_keypoints", &AKAZEOptions::save_keypoints)
    .def_readwrite("verbosity", &AKAZEOptions::verbosity)
  ;

  m.def("akaze", csfm::akaze);

  m.def("hahog", csfm::hahog,
        py::arg("image"),
        py::arg("peak_threshold") = 0.003,
        py::arg("edge_threshold") = 10,
        py::arg("target_num_features") = 0,
        py::arg("use_adaptive_suppression") = false
  );

  m.def("triangulate_bearings_dlt", csfm::TriangulateBearingsDLT);
  m.def("triangulate_bearings_midpoint", csfm::TriangulateBearingsMidpoint);

  py::class_<BundleAdjuster>(m, "BundleAdjuster")
    .def("run", &BundleAdjuster::Run)
    .def("get_perspective_camera", &BundleAdjuster::GetPerspectiveCamera)
    .def("get_brown_perspective_camera", &BundleAdjuster::GetBrownPerspectiveCamera)
    .def("get_fisheye_camera", &BundleAdjuster::GetFisheyeCamera)
    .def("get_equirectangular_camera", &BundleAdjuster::GetEquirectangularCamera)
    .def("get_shot", &BundleAdjuster::GetShot)
    .def("get_point", &BundleAdjuster::GetPoint)
    .def("add_perspective_camera", &BundleAdjuster::AddPerspectiveCamera)
    .def("add_brown_perspective_camera", &BundleAdjuster::AddBrownPerspectiveCamera)
    .def("add_fisheye_camera", &BundleAdjuster::AddFisheyeCamera)
    .def("add_equirectangular_camera", &BundleAdjuster::AddEquirectangularCamera)
    .def("add_shot", &BundleAdjuster::AddShot)
    .def("add_point", &BundleAdjuster::AddPoint)
    .def("add_observation", &BundleAdjuster::AddObservation)
    .def("add_rotation_prior", &BundleAdjuster::AddRotationPrior)
    .def("add_translation_prior", &BundleAdjuster::AddTranslationPrior)
    .def("add_position_prior", &BundleAdjuster::AddPositionPrior)
    .def("add_point_position_prior", &BundleAdjuster::AddPointPositionPrior)
    .def("add_ground_control_point_observation", &BundleAdjuster::AddGroundControlPointObservation)
    .def("set_origin_shot", &BundleAdjuster::SetOriginShot)
    .def("set_unit_translation_shot", &BundleAdjuster::SetUnitTranslationShot)
    .def("set_loss_function", &BundleAdjuster::SetLossFunction)
    .def("set_reprojection_error_sd", &BundleAdjuster::SetReprojectionErrorSD)
    .def("set_max_num_iterations", &BundleAdjuster::SetMaxNumIterations)
    .def("set_num_threads", &BundleAdjuster::SetNumThreads)
    .def("set_linear_solver_type", &BundleAdjuster::SetLinearSolverType)
    .def("set_internal_parameters_prior_sd", &BundleAdjuster::SetInternalParametersPriorSD)
    .def("set_compute_covariances", &BundleAdjuster::SetComputeCovariances)
    .def("get_covariance_estimation_valid", &BundleAdjuster::GetCovarianceEstimationValid)
    .def("set_compute_reprojection_errors", &BundleAdjuster::SetComputeReprojectionErrors)
    .def("brief_report", &BundleAdjuster::BriefReport)
    .def("full_report", &BundleAdjuster::FullReport)
  ;

  py::class_<BAPerspectiveCamera>(m, "BAPerspectiveCamera")
    .def_property("focal", &BAPerspectiveCamera::GetFocal, &BAPerspectiveCamera::SetFocal)
    .def_property("k1", &BAPerspectiveCamera::GetK1, &BAPerspectiveCamera::SetK1)
    .def_property("k2", &BAPerspectiveCamera::GetK2, &BAPerspectiveCamera::SetK2)
    .def_readwrite("constant", &BAPerspectiveCamera::constant)
    .def_readwrite("focal_prior", &BAPerspectiveCamera::focal_prior)
    .def_readwrite("id", &BAPerspectiveCamera::id)
  ;

  py::class_<BABrownPerspectiveCamera>(m, "BABrownPerspectiveCamera")
    .def_property("focal_x", &BABrownPerspectiveCamera::GetFocalX, &BABrownPerspectiveCamera::SetFocalX)
    .def_property("focal_y", &BABrownPerspectiveCamera::GetFocalY, &BABrownPerspectiveCamera::SetFocalY)
    .def_property("c_x", &BABrownPerspectiveCamera::GetCX, &BABrownPerspectiveCamera::SetCX)
    .def_property("c_y", &BABrownPerspectiveCamera::GetCY, &BABrownPerspectiveCamera::SetCY)
    .def_property("k1", &BABrownPerspectiveCamera::GetK1, &BABrownPerspectiveCamera::SetK1)
    .def_property("k2", &BABrownPerspectiveCamera::GetK2, &BABrownPerspectiveCamera::SetK2)
    .def_property("p1", &BABrownPerspectiveCamera::GetP1, &BABrownPerspectiveCamera::SetP1)
    .def_property("p2", &BABrownPerspectiveCamera::GetP2, &BABrownPerspectiveCamera::SetP2)
    .def_property("k3", &BABrownPerspectiveCamera::GetK3, &BABrownPerspectiveCamera::SetK3)
    .def_readwrite("focal_x_prior", &BABrownPerspectiveCamera::focal_x_prior)
    .def_readwrite("focal_y_prior", &BABrownPerspectiveCamera::focal_y_prior)
    .def_readwrite("c_x_prior", &BABrownPerspectiveCamera::c_x_prior)
    .def_readwrite("c_y_prior", &BABrownPerspectiveCamera::c_y_prior)
    .def_readwrite("k1_prior", &BABrownPerspectiveCamera::k1_prior)
    .def_readwrite("k2_prior", &BABrownPerspectiveCamera::k2_prior)
    .def_readwrite("p1_prior", &BABrownPerspectiveCamera::p1_prior)
    .def_readwrite("p2_prior", &BABrownPerspectiveCamera::p2_prior)
    .def_readwrite("k3_prior", &BABrownPerspectiveCamera::k3_prior)
    .def_readwrite("constant", &BABrownPerspectiveCamera::constant)
    .def_readwrite("id", &BABrownPerspectiveCamera::id)
  ;

  py::class_<BAFisheyeCamera>(m, "BAFisheyeCamera")
    .def_property("focal", &BAFisheyeCamera::GetFocal, &BAFisheyeCamera::SetFocal)
    .def_property("k1", &BAFisheyeCamera::GetK1, &BAFisheyeCamera::SetK1)
    .def_property("k2", &BAFisheyeCamera::GetK2, &BAFisheyeCamera::SetK2)
    .def_readwrite("constant", &BAFisheyeCamera::constant)
    .def_readwrite("focal_prior", &BAFisheyeCamera::focal_prior)
    .def_readwrite("id", &BAFisheyeCamera::id)
  ;

  py::class_<BAShot>(m, "BAShot")
    .def_property("rx", &BAShot::GetRX, &BAShot::SetRX)
    .def_property("ry", &BAShot::GetRY, &BAShot::SetRY)
    .def_property("rz", &BAShot::GetRZ, &BAShot::SetRZ)
    .def_property("tx", &BAShot::GetTX, &BAShot::SetTX)
    .def_property("ty", &BAShot::GetTY, &BAShot::SetTY)
    .def_property("tz", &BAShot::GetTZ, &BAShot::SetTZ)
    .def("get_covariance", &BAShot::GetCovariance)
    .def_readwrite("constant", &BAShot::constant)
    .def_readwrite("camera", &BAShot::camera)
    .def_readwrite("id", &BAShot::id)
  ;

  py::class_<BAPoint>(m, "BAPoint")
    .def_property("x", &BAPoint::GetX, &BAPoint::SetX)
    .def_property("y", &BAPoint::GetY, &BAPoint::SetY)
    .def_property("z", &BAPoint::GetZ, &BAPoint::SetZ)
    .def_readwrite("constant", &BAPoint::constant)
    .def_readwrite("reprojection_error", &BAPoint::reprojection_error)
    .def_readwrite("id", &BAPoint::id)
  ;

  py::class_<csfm::OpenMVSExporter>(m, "OpenMVSExporter")
    .def("add_camera", &csfm::OpenMVSExporter::AddCamera)
    .def("add_shot", &csfm::OpenMVSExporter::AddShot)
    .def("add_point", &csfm::OpenMVSExporter::AddPoint)
    .def("export", &csfm::OpenMVSExporter::Export)
  ;

  py::class_<csfm::DepthmapEstimatorWrapper>(m, "DepthmapEstimator")
    .def("set_depth_range", &csfm::DepthmapEstimatorWrapper::SetDepthRange)
    .def("set_patchmatch_iterations", &csfm::DepthmapEstimatorWrapper::SetPatchMatchIterations)
    .def("set_min_patch_sd", &csfm::DepthmapEstimatorWrapper::SetMinPatchSD)
    .def("add_view", &csfm::DepthmapEstimatorWrapper::AddView)
    .def("compute_patch_match", &csfm::DepthmapEstimatorWrapper::ComputePatchMatch)
    .def("compute_patch_match_sample", &csfm::DepthmapEstimatorWrapper::ComputePatchMatchSample)
    .def("compute_brute_force", &csfm::DepthmapEstimatorWrapper::ComputeBruteForce)
  ;

  py::class_<csfm::DepthmapCleanerWrapper>(m, "DepthmapCleaner")
    .def("set_same_depth_threshold", &csfm::DepthmapCleanerWrapper::SetSameDepthThreshold)
    .def("set_min_consistent_views", &csfm::DepthmapCleanerWrapper::SetMinConsistentViews)
    .def("add_view", &csfm::DepthmapCleanerWrapper::AddView)
    .def("clean", &csfm::DepthmapCleanerWrapper::Clean)
  ;

  py::class_<csfm::DepthmapPrunerWrapper>(m, "DepthmapPruner")
    .def("set_same_depth_threshold", &csfm::DepthmapPrunerWrapper::SetSameDepthThreshold)
    .def("add_view", &csfm::DepthmapPrunerWrapper::AddView)
    .def("prune", &csfm::DepthmapPrunerWrapper::Prune)
  ;

  ///////////////////////////////////
  // Reconstruction Aligment
  //
  py::class_<ReconstructionAlignment>(m, "ReconstructionAlignment")
    .def("run", &ReconstructionAlignment::Run)
    .def("get_shot", &ReconstructionAlignment::GetShot)
    .def("get_reconstruction", &ReconstructionAlignment::GetReconstruction)
    .def("add_shot", &ReconstructionAlignment::AddShot)
    .def("add_reconstruction", &ReconstructionAlignment::AddReconstruction)
    .def("add_relative_motion_constraint", &ReconstructionAlignment::AddRelativeMotionConstraint)
    .def("add_absolute_position_constraint", &ReconstructionAlignment::AddAbsolutePositionConstraint)
    .def("add_relative_absolute_position_constraint", &ReconstructionAlignment::AddRelativeAbsolutePositionConstraint)
    .def("add_common_camera_constraint", &ReconstructionAlignment::AddCommonCameraConstraint)
    .def("add_common_point_constraint", &ReconstructionAlignment::AddCommonPointConstraint)
    .def("brief_report", &ReconstructionAlignment::BriefReport)
    .def("full_report", &ReconstructionAlignment::FullReport)
  ;

  py::class_<RAShot>(m, "RAShot")
    .def_property("rx", &RAShot::GetRX, &RAShot::SetRX)
    .def_property("ry", &RAShot::GetRY, &RAShot::SetRY)
    .def_property("rz", &RAShot::GetRZ, &RAShot::SetRZ)
    .def_property("tx", &RAShot::GetTX, &RAShot::SetTX)
    .def_property("ty", &RAShot::GetTY, &RAShot::SetTY)
    .def_property("tz", &RAShot::GetTZ, &RAShot::SetTZ)
    .def_readwrite("id", &RAShot::id)
  ;

  py::class_<RAReconstruction>(m, "RAReconstruction")
    .def_property("rx", &RAReconstruction::GetRX, &RAReconstruction::SetRX)
    .def_property("ry", &RAReconstruction::GetRY, &RAReconstruction::SetRY)
    .def_property("rz", &RAReconstruction::GetRZ, &RAReconstruction::SetRZ)
    .def_property("tx", &RAReconstruction::GetTX, &RAReconstruction::SetTX)
    .def_property("ty", &RAReconstruction::GetTY, &RAReconstruction::SetTY)
    .def_property("tz", &RAReconstruction::GetTZ, &RAReconstruction::SetTZ)
    .def_property("scale", &RAReconstruction::GetScale, &RAReconstruction::SetScale)
    .def_readwrite("id", &RAReconstruction::id)
  ;

  py::class_<RARelativeMotionConstraint>(m, "RARelativeMotionConstraint")
    .def(py::init<const std::string &, const std::string &, double, double, double, double, double, double>())
    .def_readwrite("reconstruction", &RARelativeMotionConstraint::reconstruction_id)
    .def_readwrite("shot", &RARelativeMotionConstraint::shot_id)
    .def_property("rx", &RARelativeMotionConstraint::GetRX, &RARelativeMotionConstraint::SetRX)
    .def_property("ry", &RARelativeMotionConstraint::GetRY, &RARelativeMotionConstraint::SetRY)
    .def_property("rz", &RARelativeMotionConstraint::GetRZ, &RARelativeMotionConstraint::SetRZ)
    .def_property("tx", &RARelativeMotionConstraint::GetTX, &RARelativeMotionConstraint::SetTX)
    .def_property("ty", &RARelativeMotionConstraint::GetTY, &RARelativeMotionConstraint::SetTY)
    .def_property("tz", &RARelativeMotionConstraint::GetTZ, &RARelativeMotionConstraint::SetTZ)
    .def("set_scale_matrix", &RARelativeMotionConstraint::SetScaleMatrix)
  ;

}
