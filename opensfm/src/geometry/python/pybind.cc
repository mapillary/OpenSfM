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

  py::enum_<ProjectionType>(m, "ProjectionType")
    .value("PERSPECTIVE", ProjectionType::PERSPECTIVE)
    .value("BROWN", ProjectionType::BROWN)
    .value("FISHEYE", ProjectionType::FISHEYE)
    .value("DUAL", ProjectionType::DUAL)
    .value("SPHERICAL", ProjectionType::SPHERICAL)
    .export_values()
  ;

  py::class_<Camera>(m, "Camera")
  .def_static("create_perspective", &Camera::CreatePerspectiveCamera)
  .def_static("create_brown", &Camera::CreateBrownCamera)
  .def_static("create_fisheye", &Camera::CreateFisheyeCamera)
  .def_static("create_dual", &Camera::CreateDualCamera)
  .def_static("create_spherical", &Camera::CreateSphericalCamera)
  .def("project", &Camera::Project)
  .def("project_many", &Camera::ProjectMany)
  .def("pixel_bearing", &Camera::Bearing)
  .def("pixel_bearing_many", &Camera::BearingsMany)
  .def("get_K", &Camera::GetProjectionMatrix)
  .def("get_K_in_pixel_coordinates", &Camera::GetProjectionMatrixScaled)
  .def_readwrite("width", &Camera::width)
  .def_readwrite("height", &Camera::height)
  .def_readwrite("id", &Camera::id)
  .def_property("focal", &Camera::GetFocal, &Camera::SetFocal)
  .def_property("aspect_ratio", &Camera::GetAspectRatio, &Camera::SetAspectRatio)
  .def_property("distortion", &Camera::GetDistortion, &Camera::SetDistortion)
  .def_property("principal_point", &Camera::GetPrincipalPoint, &Camera::SetPrincipalPoint)
  .def_property("projection_params", &Camera::GetProjectionParams, &Camera::SetProjectionParams)
  .def_property_readonly("projection_type", &Camera::GetProjectionString)
  .def_property_readonly("k1", [](const Camera& c){return c.GetDistortion()[0];})
  .def_property_readonly("k2", [](const Camera& c){return c.GetDistortion()[1];})
  .def_property_readonly("k3", [](const Camera& c){return c.GetDistortion()[2];})
  .def_property_readonly("p1", [](const Camera& c){return c.GetDistortion()[3];})
  .def_property_readonly("p2", [](const Camera& c){return c.GetDistortion()[4];})
  .def(py::pickle(
    [](const Camera &p) {
      return py::make_tuple(p.GetProjectionParams(), p.GetDistortion(),
      p.GetFocal(), p.GetPrincipalPoint(), p.GetAspectRatio(),
      p.GetProjectionType(), p.width, p.height, p.id);
    },
    [](py::tuple t) {
      const auto projection_params = t[0].cast<Eigen::VectorXd>();
      const auto distortion = t[1].cast<Eigen::VectorXd>();
      const auto focal = t[2].cast<double>();
      const auto principal_point = t[3].cast<Eigen::Vector2d>();
      const auto aspect_ratio = t[4].cast<double>();
      const auto type = t[5].cast<ProjectionType>();
      const auto width = t[6].cast<int>();
      const auto height = t[7].cast<int>();
      const auto id = t[8].cast<std::string>();
      Camera camera = Camera::CreatePerspectiveCamera(0, 0, 0);
      switch(type){
        case ProjectionType::PERSPECTIVE:
          camera = Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]); break;
        case ProjectionType::BROWN:
          camera = Camera::CreateBrownCamera (focal, aspect_ratio, principal_point, distortion ); break;
        case ProjectionType::FISHEYE:
          camera = Camera::CreateFisheyeCamera(focal, distortion[0], distortion[1]); break;
        case ProjectionType::DUAL:
          camera = Camera::CreateDualCamera(projection_params[0], focal, distortion[0], distortion[1]); break;
        case ProjectionType::SPHERICAL:
          camera = Camera::CreateSphericalCamera(); break;
      }
      camera.width = width;
      camera.height = height;
      camera.id = id;
      return camera;
    }))
  // Python2 + copy/deepcopy + pybind11 workaround
  .def("__copy__", [](const Camera& c, const py::dict& d){ return c;}, py::return_value_policy::copy)
  .def("__deepcopy__", [](const Camera& c, const py::dict& d){ return c;}, py::return_value_policy::copy)
  ;
  m.def("compute_camera_mapping", ComputeCameraMapping);

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
