#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <foundation/python_types.h>
#include <foundation/logger.h>
#include <bundle/bundle_adjuster.h>
#include <bundle/reconstruction_alignment.h>


void BundleAdjusterRun(BundleAdjuster* bundle_adjuster) {
    py::gil_scoped_release release;
    bundle_adjuster->Run();
}


PYBIND11_MODULE(pybundle, m) {
  GLogInitializationWrapper::Instance();
  py::class_<BundleAdjuster>(m, "BundleAdjuster")
    .def(py::init())
    .def("run", &BundleAdjusterRun)
    .def("set_point_projection_loss_function", &BundleAdjuster::SetPointProjectionLossFunction)
    .def("set_relative_motion_loss_function", &BundleAdjuster::SetRelativeMotionLossFunction)
    .def("get_camera", &BundleAdjuster::GetCamera)
    .def("get_perspective_camera", &BundleAdjuster::GetPerspectiveCamera)
    .def("get_brown_perspective_camera", &BundleAdjuster::GetBrownPerspectiveCamera)
    .def("get_fisheye_camera", &BundleAdjuster::GetFisheyeCamera)
    .def("get_dual_camera", &BundleAdjuster::GetDualCamera)
    .def("get_equirectangular_camera", &BundleAdjuster::GetEquirectangularCamera)
    .def("get_shot", &BundleAdjuster::GetShot)
    .def("get_point", &BundleAdjuster::GetPoint)
    .def("set_scale_sharing", &BundleAdjuster::SetScaleSharing)
    .def("get_reconstruction", &BundleAdjuster::GetReconstruction)
    .def("add_perspective_camera", &BundleAdjuster::AddPerspectiveCamera)
    .def("add_brown_perspective_camera", &BundleAdjuster::AddBrownPerspectiveCamera)
    .def("add_fisheye_camera", &BundleAdjuster::AddFisheyeCamera)
    .def("add_dual_camera", &BundleAdjuster::AddDualCamera)
    .def("add_equirectangular_camera", &BundleAdjuster::AddEquirectangularCamera)
    .def("add_shot", &BundleAdjuster::AddShot)
    .def("add_point", &BundleAdjuster::AddPoint)
    .def("add_reconstruction", &BundleAdjuster::AddReconstruction)
    .def("add_reconstruction_shot", &BundleAdjuster::AddReconstructionShot)
    .def("add_point_projection_observation", &BundleAdjuster::AddPointProjectionObservation)
    .def("add_relative_motion", &BundleAdjuster::AddRelativeMotion)
    .def("add_relative_similarity", &BundleAdjuster::AddRelativeSimilarity)
    .def("add_relative_rotation", &BundleAdjuster::AddRelativeRotation)
    .def("add_common_position", &BundleAdjuster::AddCommonPosition)
    .def("add_absolute_position", &BundleAdjuster::AddAbsolutePosition)
    .def("add_absolute_up_vector", &BundleAdjuster::AddAbsoluteUpVector)
    .def("add_absolute_pan", &BundleAdjuster::AddAbsolutePan)
    .def("add_absolute_tilt", &BundleAdjuster::AddAbsoluteTilt)
    .def("add_absolute_roll", &BundleAdjuster::AddAbsoluteRoll)
    .def("add_rotation_prior", &BundleAdjuster::AddRotationPrior)
    .def("add_translation_prior", &BundleAdjuster::AddTranslationPrior)
    .def("add_position_prior", &BundleAdjuster::AddPositionPrior)
    .def("add_point_position_prior", &BundleAdjuster::AddPointPositionPrior)
    .def("set_origin_shot", &BundleAdjuster::SetOriginShot)
    .def("set_unit_translation_shot", &BundleAdjuster::SetUnitTranslationShot)
    .def("add_point_position_shot", &BundleAdjuster::AddPointPositionShot)
    .def("add_point_position_world", &BundleAdjuster::AddPointPositionWorld)
    .def("add_linear_motion", &BundleAdjuster::AddLinearMotion)
    .def("set_internal_parameters_prior_sd", &BundleAdjuster::SetInternalParametersPriorSD)
    .def("set_compute_covariances", &BundleAdjuster::SetComputeCovariances)
    .def("get_covariance_estimation_valid", &BundleAdjuster::GetCovarianceEstimationValid)
    .def("set_compute_reprojection_errors", &BundleAdjuster::SetComputeReprojectionErrors)
    .def("set_max_num_iterations", &BundleAdjuster::SetMaxNumIterations)
    .def("set_adjust_absolute_position_std", &BundleAdjuster::SetAdjustAbsolutePositionStd)
    .def("set_num_threads", &BundleAdjuster::SetNumThreads)
    .def("set_use_new", &BundleAdjuster::SetUseNew)
    .def("set_linear_solver_type", &BundleAdjuster::SetLinearSolverType)
    .def("brief_report", &BundleAdjuster::BriefReport)
    .def("full_report", &BundleAdjuster::FullReport)
  ;

  py::enum_<PositionConstraintType>(m, "PositionConstraintType")
        .value("X", PositionConstraintType::X)
        .value("Y", PositionConstraintType::Y)
        .value("Z", PositionConstraintType::Z)
        .value("XY", PositionConstraintType::XY)
        .value("XYZ", PositionConstraintType::XYZ)
        .export_values()
  ;

  py::class_<BAPerspectiveCamera>(m, "BAPerspectiveCamera")
    .def(py::init())
    .def_property("focal", &BAPerspectiveCamera::GetFocal, &BAPerspectiveCamera::SetFocal)
    .def_property("k1", &BAPerspectiveCamera::GetK1, &BAPerspectiveCamera::SetK1)
    .def_property("k2", &BAPerspectiveCamera::GetK2, &BAPerspectiveCamera::SetK2)
    .def_readwrite("constant", &BAPerspectiveCamera::constant)
    .def_readwrite("focal_prior", &BAPerspectiveCamera::focal_prior)
    .def_readwrite("id", &BAPerspectiveCamera::id)
  ;

  py::class_<BABrownPerspectiveCamera>(m, "BABrownPerspectiveCamera")
    .def(py::init())
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
    .def(py::init())
    .def_property("focal", &BAFisheyeCamera::GetFocal, &BAFisheyeCamera::SetFocal)
    .def_property("k1", &BAFisheyeCamera::GetK1, &BAFisheyeCamera::SetK1)
    .def_property("k2", &BAFisheyeCamera::GetK2, &BAFisheyeCamera::SetK2)
    .def_readwrite("constant", &BAFisheyeCamera::constant)
    .def_readwrite("focal_prior", &BAFisheyeCamera::focal_prior)
    .def_readwrite("id", &BAFisheyeCamera::id)
  ;

  py::class_<BADualCamera>(m, "BADualCamera")
    .def(py::init())
    .def_property("focal", &BADualCamera::GetFocal, &BADualCamera::SetFocal)
    .def_property("k1", &BADualCamera::GetK1, &BADualCamera::SetK1)
    .def_property("k2", &BADualCamera::GetK2, &BADualCamera::SetK2)
    .def_property("transition", &BADualCamera::GetTransition, &BADualCamera::SetTransition)
    .def_readwrite("constant", &BADualCamera::constant)
    .def_readwrite("focal_prior", &BADualCamera::focal_prior)
    .def_readwrite("id", &BADualCamera::id)
  ;

  py::class_<BAShot>(m, "BAShot")
    .def(py::init())
    .def_property_readonly("r", &BAShot::GetRotation)
    .def_property_readonly("t", &BAShot::GetTranslation)
    .def_readwrite("id", &BAShot::id)
    .def_readwrite("camera", &BAShot::camera)
    .def("get_covariance_inv_param", &BAShot::GetCovarianceInvParam)
  ;

  py::class_<BAReconstruction>(m, "BAReconstruction")
    .def(py::init())
    .def("get_scale", &BAReconstruction::GetScale)
    .def("set_scale", &BAReconstruction::SetScale)
    .def_readwrite("id", &BAReconstruction::id)
  ;

  py::class_<BAPoint>(m, "BAPoint")
    .def(py::init())
    .def_property("p", &BAPoint::GetPoint, &BAPoint::SetPoint)
    .def_readwrite("id", &BAPoint::id)
    .def_readwrite("reprojection_errors", &BAPoint::reprojection_errors)
  ;

  py::class_<BARelativeMotion>(m, "BARelativeMotion")
    .def(py::init<const std::string &, const std::string &, 
	        const std::string &, const std::string &, 
          const Eigen::Vector3d &, const Eigen::Vector3d &,
          double>())
    .def_readwrite("reconstruction_i", &BARelativeMotion::reconstruction_id_i)
    .def_readwrite("shot_i", &BARelativeMotion::shot_id_i)
    .def_readwrite("reconstruction_j", &BARelativeMotion::reconstruction_id_j)
    .def_readwrite("shot_j", &BARelativeMotion::shot_id_j)
    .def_property("r", &BARelativeMotion::GetRotation, &BARelativeMotion::SetRotation)
    .def_property("t", &BARelativeMotion::GetTranslation, &BARelativeMotion::SetTranslation)
    .def("set_scale_matrix", &BARelativeMotion::SetScaleMatrix)
  ;

  py::class_<BARelativeSimilarity>(m, "BARelativeSimilarity")
    .def(py::init<const std::string &, const std::string &, 
	        const std::string &, const std::string &, 
	        const Eigen::Vector3d &, const Eigen::Vector3d &,
          double, double>())
    .def_readwrite("scale", &BARelativeSimilarity::scale)
    .def("set_scale_matrix", &BARelativeSimilarity::SetScaleMatrix)
  ;
  
  py::class_<BARelativeSimilarityCovariance>(m, "BARelativeSimilarityCovariance")
    .def(py::init())
    .def("add_point", &BARelativeSimilarityCovariance::AddPoint)
    .def("compute", &BARelativeSimilarityCovariance::Compute)
    .def("get_covariance", &BARelativeSimilarityCovariance::GetCovariance)
  ;

  py::class_<BARelativeRotation>(m, "BARelativeRotation")
    .def(py::init<const std::string &, const std::string &, const Eigen::Vector3d &>())
    .def_readwrite("shot_i", &BARelativeRotation::shot_id_i)
    .def_readwrite("shot_j", &BARelativeRotation::shot_id_j)
    .def_property("r", &BARelativeRotation::GetRotation, &BARelativeRotation::SetRotation)
    .def("set_scale_matrix", &BARelativeRotation::SetScaleMatrix)
  ;

  ///////////////////////////////////
  // Reconstruction Aligment
  //
  py::class_<ReconstructionAlignment>(m, "ReconstructionAlignment")
    .def(py::init())
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
    .def(py::init())
    .def_property("rx", &RAShot::GetRX, &RAShot::SetRX)
    .def_property("ry", &RAShot::GetRY, &RAShot::SetRY)
    .def_property("rz", &RAShot::GetRZ, &RAShot::SetRZ)
    .def_property("tx", &RAShot::GetTX, &RAShot::SetTX)
    .def_property("ty", &RAShot::GetTY, &RAShot::SetTY)
    .def_property("tz", &RAShot::GetTZ, &RAShot::SetTZ)
    .def_readwrite("id", &RAShot::id)
  ;

  py::class_<RAReconstruction>(m, "RAReconstruction")
    .def(py::init())
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
