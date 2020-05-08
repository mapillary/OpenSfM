#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <glog/logging.h>

#include <geometry/essential.h>
#include <geometry/camera.h>
#include <geometry/relative_pose.h>
#include <geometry/absolute_pose.h>
#include <geometry/triangulation.h>
#include <foundation/types.h>


PYBIND11_MODULE(pygeometry, m) {

  py::class_<Camera>(m, "Camera")
  .def_static("create_perspective", &Camera::CreatePerspective)
  .def_static("create_brown", &Camera::CreateBrownCamera)
  .def_static("create_fisheye", &Camera::CreateFisheyeCamera)
  .def_static("create_dual", &Camera::CreateDualCamera)
  .def_static("create_spherical", &Camera::CreateSphericalCamera)
  .def("project", &Camera::Project)
  .def("project_many", &Camera::ProjectMany)
  .def("pixel_bearing", &Camera::Bearing)
  .def("pixel_bearing_many", &Camera::BearingsMany)
  .def_readwrite("width", &Camera::width)
  .def_readwrite("height", &Camera::height)
  .def_readwrite("id", &Camera::id)
  .def_property("focal", &Camera::GetFocal, &Camera::SetFocal)
  .def_property("aspec_ratio", &Camera::GetAspectRatio, &Camera::SetAspectRatio)
  .def_property("distortion", &Camera::GetDistortion, &Camera::SetDistortion)
  .def_property("principal_point", &Camera::GetPrincipalPoint, &Camera::SetPrincipalPoint)
  ;

  m.def("triangulate_bearings_dlt", geometry::TriangulateBearingsDLT);
  m.def("triangulate_bearings_midpoint", geometry::TriangulateBearingsMidpoint);
  m.def("triangulate_two_bearings_midpoint", geometry::TriangulateTwoBearingsMidpointSolve<double>);
  m.def("triangulate_two_bearings_midpoint_many", geometry::TriangulateTwoBearingsMidpointMany);
  m.def("essential_five_points", geometry::EssentialFivePoints);
  m.def("absolute_pose_three_points", geometry::AbsolutePoseThreePoints);
  m.def("absolute_pose_n_points", geometry::AbsolutePoseNPoints);
  m.def("absolute_pose_n_points_known_rotation", geometry::AbsolutePoseNPointsKnownRotation);
  m.def("essential_n_points", geometry::EssentialNPoints);
  m.def("relative_pose_from_essential", geometry::RelativePoseFromEssential);
  m.def("relative_rotation_n_points", geometry::RelativeRotationNPoints);
  m.def("relative_pose_refinement", geometry::RelativePoseRefinement);
}
