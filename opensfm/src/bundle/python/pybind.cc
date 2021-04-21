#include <bundle/bundle_adjuster.h>
#include <bundle/reconstruction_alignment.h>
#include <foundation/python_types.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

PYBIND11_MODULE(pybundle, m) {
  py::class_<bundle::BundleAdjuster>(m, "BundleAdjuster")
      .def(py::init())
      .def("run", &bundle::BundleAdjuster::Run,
           py::call_guard<py::gil_scoped_release>())
      .def("set_point_projection_loss_function",
           &bundle::BundleAdjuster::SetPointProjectionLossFunction)
      .def("set_relative_motion_loss_function",
           &bundle::BundleAdjuster::SetRelativeMotionLossFunction)
      .def("add_camera", &bundle::BundleAdjuster::AddCamera)
      .def("get_camera", &bundle::BundleAdjuster::GetCamera)
      .def("get_shot", &bundle::BundleAdjuster::GetShot)
      .def("get_point", &bundle::BundleAdjuster::GetPoint)
      .def("set_scale_sharing", &bundle::BundleAdjuster::SetScaleSharing)
      .def("get_reconstruction", &bundle::BundleAdjuster::GetReconstruction)
      .def("add_shot", &bundle::BundleAdjuster::AddShot)
      .def("add_point", &bundle::BundleAdjuster::AddPoint)
      .def("add_reconstruction", &bundle::BundleAdjuster::AddReconstruction)
      .def("add_reconstruction_shot",
           &bundle::BundleAdjuster::AddReconstructionShot)
      .def("add_point_projection_observation",
           &bundle::BundleAdjuster::AddPointProjectionObservation)
      .def("add_relative_motion", &bundle::BundleAdjuster::AddRelativeMotion)
      .def("add_relative_similarity",
           &bundle::BundleAdjuster::AddRelativeSimilarity)
      .def("add_relative_rotation",
           &bundle::BundleAdjuster::AddRelativeRotation)
      .def("add_common_position", &bundle::BundleAdjuster::AddCommonPosition)
      .def("add_absolute_position",
           &bundle::BundleAdjuster::AddAbsolutePosition)
      .def("add_heatmap", &bundle::BundleAdjuster::AddHeatmap)
      .def("add_absolute_position_heatmap",
           &bundle::BundleAdjuster::AddAbsolutePositionHeatmap)
      .def("add_absolute_up_vector",
           &bundle::BundleAdjuster::AddAbsoluteUpVector)
      .def("add_absolute_pan", &bundle::BundleAdjuster::AddAbsolutePan)
      .def("add_absolute_tilt", &bundle::BundleAdjuster::AddAbsoluteTilt)
      .def("add_absolute_roll", &bundle::BundleAdjuster::AddAbsoluteRoll)
      .def("add_rotation_prior", &bundle::BundleAdjuster::AddRotationPrior)
      .def("add_translation_prior",
           &bundle::BundleAdjuster::AddTranslationPrior)
      .def("add_position_prior", &bundle::BundleAdjuster::AddPositionPrior)
      .def("add_point_position_prior",
           &bundle::BundleAdjuster::AddPointPositionPrior)
      .def("set_origin_shot", &bundle::BundleAdjuster::SetOriginShot)
      .def("set_unit_translation_shot",
           &bundle::BundleAdjuster::SetUnitTranslationShot)
      .def("add_point_position_shot",
           &bundle::BundleAdjuster::AddPointPositionShot)
      .def("add_point_position_world",
           &bundle::BundleAdjuster::AddPointPositionWorld)
      .def("add_linear_motion", &bundle::BundleAdjuster::AddLinearMotion)
      .def("set_internal_parameters_prior_sd",
           &bundle::BundleAdjuster::SetInternalParametersPriorSD)
      .def("set_compute_covariances",
           &bundle::BundleAdjuster::SetComputeCovariances)
      .def("get_covariance_estimation_valid",
           &bundle::BundleAdjuster::GetCovarianceEstimationValid)
      .def("set_compute_reprojection_errors",
           &bundle::BundleAdjuster::SetComputeReprojectionErrors)
      .def("set_max_num_iterations",
           &bundle::BundleAdjuster::SetMaxNumIterations)
      .def("set_adjust_absolute_position_std",
           &bundle::BundleAdjuster::SetAdjustAbsolutePositionStd)
      .def("set_num_threads", &bundle::BundleAdjuster::SetNumThreads)
      .def("set_use_analytic_derivatives",
           &bundle::BundleAdjuster::SetUseAnalyticDerivatives)
      .def("set_linear_solver_type",
           &bundle::BundleAdjuster::SetLinearSolverType)
      .def("brief_report", &bundle::BundleAdjuster::BriefReport)
      .def("full_report", &bundle::BundleAdjuster::FullReport);

  py::enum_<bundle::PositionConstraintType>(m, "PositionConstraintType")
      .value("X", bundle::PositionConstraintType::X)
      .value("Y", bundle::PositionConstraintType::Y)
      .value("Z", bundle::PositionConstraintType::Z)
      .value("XY", bundle::PositionConstraintType::XY)
      .value("XYZ", bundle::PositionConstraintType::XYZ)
      .export_values();

  py::class_<bundle::Shot>(m, "Shot")
      .def_property_readonly(
          "r",
          [](const bundle::Shot &s) {
            return s.GetPose()->GetValue().RotationWorldToCameraMin();
          })
      .def_property_readonly(
          "t",
          [](const bundle::Shot &s) {
            return s.GetPose()->GetValue().TranslationWorldToCamera();
          })
      .def_property_readonly("id",
                             [](const bundle::Shot &s) { return s.GetID(); })
      .def_property_readonly(
          "camera", [](const bundle::Shot &s) { return s.GetCamera(); })
      .def("get_covariance_inv_param",
           [](const bundle::Shot &s) { return s.GetPose()->GetCovariance(); });

  py::class_<bundle::Reconstruction>(m, "Reconstruction")
      .def(py::init())
      .def("get_scale", &bundle::Reconstruction::GetScale)
      .def("set_scale", &bundle::Reconstruction::SetScale)
      .def_readwrite("id", &bundle::Reconstruction::id);

  py::class_<bundle::Point>(m, "Point")
      .def(py::init())
      .def_property("p", &bundle::Point::GetPoint, &bundle::Point::SetPoint)
      .def_readwrite("id", &bundle::Point::id)
      .def_readwrite("reprojection_errors",
                     &bundle::Point::reprojection_errors);

  py::class_<bundle::RelativeMotion>(m, "RelativeMotion")
      .def(py::init<const std::string &, const std::string &,
                    const std::string &, const std::string &,
                    const Eigen::Vector3d &, const Eigen::Vector3d &, double>())
      .def_readwrite("reconstruction_i",
                     &bundle::RelativeMotion::reconstruction_id_i)
      .def_readwrite("shot_i", &bundle::RelativeMotion::shot_id_i)
      .def_readwrite("reconstruction_j",
                     &bundle::RelativeMotion::reconstruction_id_j)
      .def_readwrite("shot_j", &bundle::RelativeMotion::shot_id_j)
      .def_property("r", &bundle::RelativeMotion::GetRotation,
                    &bundle::RelativeMotion::SetRotation)
      .def_property("t", &bundle::RelativeMotion::GetTranslation,
                    &bundle::RelativeMotion::SetTranslation)
      .def("set_scale_matrix", &bundle::RelativeMotion::SetScaleMatrix);

  py::class_<bundle::RelativeSimilarity>(m, "RelativeSimilarity")
      .def(py::init<const std::string &, const std::string &,
                    const std::string &, const std::string &,
                    const Eigen::Vector3d &, const Eigen::Vector3d &, double,
                    double>())
      .def_readwrite("scale", &bundle::RelativeSimilarity::scale)
      .def("set_scale_matrix", &bundle::RelativeSimilarity::SetScaleMatrix);

  py::class_<bundle::RelativeSimilarityCovariance>(
      m, "RelativeSimilarityCovariance")
      .def(py::init())
      .def("add_point", &bundle::RelativeSimilarityCovariance::AddPoint)
      .def("compute", &bundle::RelativeSimilarityCovariance::Compute)
      .def("get_covariance",
           &bundle::RelativeSimilarityCovariance::GetCovariance);

  py::class_<bundle::RelativeRotation>(m, "RelativeRotation")
      .def(py::init<const std::string &, const std::string &,
                    const Eigen::Vector3d &>())
      .def_readwrite("shot_i", &bundle::RelativeRotation::shot_id_i)
      .def_readwrite("shot_j", &bundle::RelativeRotation::shot_id_j)
      .def_property("r", &bundle::RelativeRotation::GetRotation,
                    &bundle::RelativeRotation::SetRotation)
      .def("set_scale_matrix", &bundle::RelativeRotation::SetScaleMatrix);

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
