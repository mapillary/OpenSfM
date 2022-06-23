#include <bundle/bundle_adjuster.h>
#include <bundle/reconstruction_alignment.h>
#include <foundation/python_types.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

PYBIND11_MODULE(pybundle, m) {
  py::module::import("opensfm.pygeometry");

  py::class_<bundle::RelativeMotion>(m, "RelativeMotion")
      .def(py::init<const std::string &, const std::string &,
                    const Eigen::Vector3d &, const Eigen::Vector3d &, double,
                    double, bool>())
      .def_readwrite("rig_instance_i",
                     &bundle::RelativeMotion::rig_instance_id_i)
      .def_readwrite("rig_instance_j",
                     &bundle::RelativeMotion::rig_instance_id_j)
      .def("set_scale_matrix", &bundle::RelativeMotion::SetScaleMatrix);

  py::class_<bundle::RelativeRotation>(m, "RelativeRotation")
      .def(py::init<const std::string &, const std::string &,
                    const Eigen::Vector3d &>())
      .def_readwrite("shot_i", &bundle::RelativeRotation::shot_id_i)
      .def_readwrite("shot_j", &bundle::RelativeRotation::shot_id_j)
      .def_property("r", &bundle::RelativeRotation::GetRotation,
                    &bundle::RelativeRotation::SetRotation)
      .def("set_scale_matrix", &bundle::RelativeRotation::SetScaleMatrix);

  py::class_<bundle::Reconstruction>(m, "Reconstruction")
      .def(py::init())
      .def("get_scale", &bundle::Reconstruction::GetScale)
      .def("set_scale", &bundle::Reconstruction::SetScale)
      .def_readwrite("id", &bundle::Reconstruction::id);

  py::class_<bundle::Point>(m, "Point")
      .def_property_readonly(
          "p", [](const bundle::Point &p) { return p.GetValue(); })
      .def_property_readonly("id",
                             [](const bundle::Point &p) { return p.GetID(); })
      .def_readwrite("reprojection_errors",
                     &bundle::Point::reprojection_errors);
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
      .def("add_rig_camera", &bundle::BundleAdjuster::AddRigCamera)
      .def("get_rig_camera_pose",
           [](const bundle::BundleAdjuster &ba,
              const std::string &rig_camera_id) {
             return ba.GetRigCamera(rig_camera_id).GetValue();
           })
      .def("add_rig_instance", &bundle::BundleAdjuster::AddRigInstance)
      .def("get_rig_instance_pose",
           [](const bundle::BundleAdjuster &ba,
              const std::string &rig_instance_id) {
             return ba.GetRigInstance(rig_instance_id).GetValue();
           })
      .def("add_rig_instance_position_prior",
           &bundle::BundleAdjuster::AddRigInstancePositionPrior)
      .def("set_scale_sharing", &bundle::BundleAdjuster::SetScaleSharing)
      .def("get_reconstruction", &bundle::BundleAdjuster::GetReconstruction)
      .def("add_point", &bundle::BundleAdjuster::AddPoint)
      .def("add_point_prior", &bundle::BundleAdjuster::AddPointPrior)
      .def("get_point", &bundle::BundleAdjuster::GetPoint)
      .def("has_point", &bundle::BundleAdjuster::HasPoint)
      .def("add_reconstruction", &bundle::BundleAdjuster::AddReconstruction)
      .def("add_reconstruction_instance",
           &bundle::BundleAdjuster::AddReconstructionInstance)
      .def("add_point_projection_observation",
           &bundle::BundleAdjuster::AddPointProjectionObservation)
      .def("add_relative_motion", &bundle::BundleAdjuster::AddRelativeMotion)
      .def("add_relative_rotation",
           &bundle::BundleAdjuster::AddRelativeRotation)
      .def("add_common_position", &bundle::BundleAdjuster::AddCommonPosition)
      .def("add_heatmap", &bundle::BundleAdjuster::AddHeatmap)
      .def("add_absolute_position_heatmap",
           &bundle::BundleAdjuster::AddAbsolutePositionHeatmap)
      .def("add_absolute_up_vector",
           &bundle::BundleAdjuster::AddAbsoluteUpVector)
      .def("add_absolute_pan", &bundle::BundleAdjuster::AddAbsolutePan)
      .def("add_absolute_tilt", &bundle::BundleAdjuster::AddAbsoluteTilt)
      .def("add_absolute_roll", &bundle::BundleAdjuster::AddAbsoluteRoll)
      .def("add_linear_motion", &bundle::BundleAdjuster::AddLinearMotion)
      .def("set_gauge_fix_shots", &bundle::BundleAdjuster::SetGaugeFixShots)
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

  ///////////////////////////////////
  // Reconstruction Alignment
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
