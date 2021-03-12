#include <bundle/bundle_adjuster.h>
#include <bundle/reconstruction_alignment.h>
#include <foundation/python_types.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

PYBIND11_MODULE(pybundle, m) {
  py::class_<BundleAdjuster>(m, "BundleAdjuster")
      .def(py::init())
      .def("run", &BundleAdjuster::Run,
           py::call_guard<py::gil_scoped_release>())
      .def("set_point_projection_loss_function",
           &BundleAdjuster::SetPointProjectionLossFunction)
      .def("set_relative_motion_loss_function",
           &BundleAdjuster::SetRelativeMotionLossFunction)
      .def("add_camera", &BundleAdjuster::AddCamera)
      .def("get_camera", &BundleAdjuster::GetCamera)
      .def("get_shot", &BundleAdjuster::GetShot)
      .def("get_point", &BundleAdjuster::GetPoint)
      .def("set_scale_sharing", &BundleAdjuster::SetScaleSharing)
      .def("get_reconstruction", &BundleAdjuster::GetReconstruction)
      .def("add_shot", &BundleAdjuster::AddShot)
      .def("add_point", &BundleAdjuster::AddPoint)
      .def("add_reconstruction", &BundleAdjuster::AddReconstruction)
      .def("add_reconstruction_shot", &BundleAdjuster::AddReconstructionShot)
      .def("add_point_projection_observation",
           &BundleAdjuster::AddPointProjectionObservation)
      .def("add_relative_motion", &BundleAdjuster::AddRelativeMotion)
      .def("add_relative_similarity", &BundleAdjuster::AddRelativeSimilarity)
      .def("add_relative_rotation", &BundleAdjuster::AddRelativeRotation)
      .def("add_common_position", &BundleAdjuster::AddCommonPosition)
      .def("add_absolute_position", &BundleAdjuster::AddAbsolutePosition)
      .def("add_heatmap", &BundleAdjuster::AddHeatmap)
      .def("add_absolute_position_heatmap",
           &BundleAdjuster::AddAbsolutePositionHeatmap)
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
      .def("set_internal_parameters_prior_sd",
           &BundleAdjuster::SetInternalParametersPriorSD)
      .def("set_compute_covariances", &BundleAdjuster::SetComputeCovariances)
      .def("get_covariance_estimation_valid",
           &BundleAdjuster::GetCovarianceEstimationValid)
      .def("set_compute_reprojection_errors",
           &BundleAdjuster::SetComputeReprojectionErrors)
      .def("set_max_num_iterations", &BundleAdjuster::SetMaxNumIterations)
      .def("set_adjust_absolute_position_std",
           &BundleAdjuster::SetAdjustAbsolutePositionStd)
      .def("set_num_threads", &BundleAdjuster::SetNumThreads)
      .def("set_use_analytic_derivatives",
           &BundleAdjuster::SetUseAnalyticDerivatives)
      .def("set_linear_solver_type", &BundleAdjuster::SetLinearSolverType)
      .def("brief_report", &BundleAdjuster::BriefReport)
      .def("full_report", &BundleAdjuster::FullReport);

  py::enum_<PositionConstraintType>(m, "PositionConstraintType")
      .value("X", PositionConstraintType::X)
      .value("Y", PositionConstraintType::Y)
      .value("Z", PositionConstraintType::Z)
      .value("XY", PositionConstraintType::XY)
      .value("XYZ", PositionConstraintType::XYZ)
      .export_values();

  py::class_<BAShot>(m, "BAShot")
      .def_property_readonly(
          "r",
          [](const BAShot &s) {
            return s.GetPose()->GetValue().RotationWorldToCameraMin();
          })
      .def_property_readonly(
          "t",
          [](const BAShot &s) {
            return s.GetPose()->GetValue().TranslationWorldToCamera();
          })
      .def_property_readonly("id", [](const BAShot &s) { return s.GetID(); })
      .def_property_readonly("camera",
                             [](const BAShot &s) { return s.GetCamera(); })
      .def("get_covariance_inv_param",
           [](const BAShot &s) { return s.GetPose()->GetCovariance(); });

  py::class_<BAReconstruction>(m, "BAReconstruction")
      .def(py::init())
      .def("get_scale", &BAReconstruction::GetScale)
      .def("set_scale", &BAReconstruction::SetScale)
      .def_readwrite("id", &BAReconstruction::id);

  py::class_<BAPoint>(m, "BAPoint")
      .def(py::init())
      .def_property("p", &BAPoint::GetPoint, &BAPoint::SetPoint)
      .def_readwrite("id", &BAPoint::id)
      .def_readwrite("reprojection_errors", &BAPoint::reprojection_errors);

  py::class_<BARelativeMotion>(m, "BARelativeMotion")
      .def(py::init<const std::string &, const std::string &,
                    const std::string &, const std::string &,
                    const Eigen::Vector3d &, const Eigen::Vector3d &, double>())
      .def_readwrite("reconstruction_i", &BARelativeMotion::reconstruction_id_i)
      .def_readwrite("shot_i", &BARelativeMotion::shot_id_i)
      .def_readwrite("reconstruction_j", &BARelativeMotion::reconstruction_id_j)
      .def_readwrite("shot_j", &BARelativeMotion::shot_id_j)
      .def_property("r", &BARelativeMotion::GetRotation,
                    &BARelativeMotion::SetRotation)
      .def_property("t", &BARelativeMotion::GetTranslation,
                    &BARelativeMotion::SetTranslation)
      .def("set_scale_matrix", &BARelativeMotion::SetScaleMatrix);

  py::class_<BARelativeSimilarity>(m, "BARelativeSimilarity")
      .def(py::init<const std::string &, const std::string &,
                    const std::string &, const std::string &,
                    const Eigen::Vector3d &, const Eigen::Vector3d &, double,
                    double>())
      .def_readwrite("scale", &BARelativeSimilarity::scale)
      .def("set_scale_matrix", &BARelativeSimilarity::SetScaleMatrix);

  py::class_<BARelativeSimilarityCovariance>(m,
                                             "BARelativeSimilarityCovariance")
      .def(py::init())
      .def("add_point", &BARelativeSimilarityCovariance::AddPoint)
      .def("compute", &BARelativeSimilarityCovariance::Compute)
      .def("get_covariance", &BARelativeSimilarityCovariance::GetCovariance);

  py::class_<BARelativeRotation>(m, "BARelativeRotation")
      .def(py::init<const std::string &, const std::string &,
                    const Eigen::Vector3d &>())
      .def_readwrite("shot_i", &BARelativeRotation::shot_id_i)
      .def_readwrite("shot_j", &BARelativeRotation::shot_id_j)
      .def_property("r", &BARelativeRotation::GetRotation,
                    &BARelativeRotation::SetRotation)
      .def("set_scale_matrix", &BARelativeRotation::SetScaleMatrix);

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
      .def("add_relative_motion_constraint",
           &ReconstructionAlignment::AddRelativeMotionConstraint)
      .def("add_absolute_position_constraint",
           &ReconstructionAlignment::AddAbsolutePositionConstraint)
      .def("add_relative_absolute_position_constraint",
           &ReconstructionAlignment::AddRelativeAbsolutePositionConstraint)
      .def("add_common_camera_constraint",
           &ReconstructionAlignment::AddCommonCameraConstraint)
      .def("add_common_point_constraint",
           &ReconstructionAlignment::AddCommonPointConstraint)
      .def("brief_report", &ReconstructionAlignment::BriefReport)
      .def("full_report", &ReconstructionAlignment::FullReport);

  py::class_<RAShot>(m, "RAShot")
      .def(py::init())
      .def_property("rx", &RAShot::GetRX, &RAShot::SetRX)
      .def_property("ry", &RAShot::GetRY, &RAShot::SetRY)
      .def_property("rz", &RAShot::GetRZ, &RAShot::SetRZ)
      .def_property("tx", &RAShot::GetTX, &RAShot::SetTX)
      .def_property("ty", &RAShot::GetTY, &RAShot::SetTY)
      .def_property("tz", &RAShot::GetTZ, &RAShot::SetTZ)
      .def_readwrite("id", &RAShot::id);

  py::class_<RAReconstruction>(m, "RAReconstruction")
      .def(py::init())
      .def_property("rx", &RAReconstruction::GetRX, &RAReconstruction::SetRX)
      .def_property("ry", &RAReconstruction::GetRY, &RAReconstruction::SetRY)
      .def_property("rz", &RAReconstruction::GetRZ, &RAReconstruction::SetRZ)
      .def_property("tx", &RAReconstruction::GetTX, &RAReconstruction::SetTX)
      .def_property("ty", &RAReconstruction::GetTY, &RAReconstruction::SetTY)
      .def_property("tz", &RAReconstruction::GetTZ, &RAReconstruction::SetTZ)
      .def_property("scale", &RAReconstruction::GetScale,
                    &RAReconstruction::SetScale)
      .def_readwrite("id", &RAReconstruction::id);

  py::class_<RARelativeMotionConstraint>(m, "RARelativeMotionConstraint")
      .def(py::init<const std::string &, const std::string &, double, double,
                    double, double, double, double>())
      .def_readwrite("reconstruction",
                     &RARelativeMotionConstraint::reconstruction_id)
      .def_readwrite("shot", &RARelativeMotionConstraint::shot_id)
      .def_property("rx", &RARelativeMotionConstraint::GetRX,
                    &RARelativeMotionConstraint::SetRX)
      .def_property("ry", &RARelativeMotionConstraint::GetRY,
                    &RARelativeMotionConstraint::SetRY)
      .def_property("rz", &RARelativeMotionConstraint::GetRZ,
                    &RARelativeMotionConstraint::SetRZ)
      .def_property("tx", &RARelativeMotionConstraint::GetTX,
                    &RARelativeMotionConstraint::SetTX)
      .def_property("ty", &RARelativeMotionConstraint::GetTY,
                    &RARelativeMotionConstraint::SetTY)
      .def_property("tz", &RARelativeMotionConstraint::GetTZ,
                    &RARelativeMotionConstraint::SetTZ)
      .def("set_scale_matrix", &RARelativeMotionConstraint::SetScaleMatrix);
}
