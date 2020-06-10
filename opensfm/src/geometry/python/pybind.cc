#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <glog/logging.h>

#include <geometry/essential.h>
#include <geometry/camera.h>
#include <geometry/relative_pose.h>
#include <geometry/absolute_pose.h>
#include <geometry/triangulation.h>
#include <foundation/python_types.h>


PYBIND11_MODULE(pygeometry, m) {

  py::enum_<ProjectionType>(m, "ProjectionType")
    .value("PERSPECTIVE", ProjectionType::PERSPECTIVE)
    .value("BROWN", ProjectionType::BROWN)
    .value("FISHEYE", ProjectionType::FISHEYE)
    .value("DUAL", ProjectionType::DUAL)
    .value("SPHERICAL", ProjectionType::SPHERICAL)
    .export_values()
  ;

  py::enum_<Camera::Parameters>(m, "CameraParameters")
    .value("focal", Camera::Parameters::Focal)
    .value("aspect_ratio", Camera::Parameters::AspectRatio)
    .value("k1", Camera::Parameters::K1)
    .value("k2", Camera::Parameters::K2)
    .value("k3", Camera::Parameters::K3)
    .value("p1", Camera::Parameters::P1)
    .value("p2", Camera::Parameters::P2)
    .value("transition", Camera::Parameters::Transition)
    .export_values()
  ;

  py::class_<Camera>(m, "Camera")
      .def("undistort_image_coordinate", &Camera::UndistortImageCoordinates)
      .def("undistort_many", &Camera::UndistortImageCoordinatesMany)
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
    .def("get_K_in_pixel_coordinates", (Mat3d (Camera::*)(int, int) const)&Camera::GetProjectionMatrixScaled)
    .def("get_K_in_pixel_coordinates", (Mat3d (Camera::*)() const)&Camera::GetProjectionMatrixScaled)
    .def("set_parameter_value", &Camera::SetParameterValue)
    .def("get_parameters_values", &Camera::GetParametersValues)
    .def("get_parameters_types", &Camera::GetParametersTypes)
    .def("get_parameters_map", &Camera::GetParametersMap)
    .def_readwrite("width", &Camera::width)
    .def_readwrite("height", &Camera::height)
    .def_readwrite("id", &Camera::id)
    .def_property(
        "focal",
        [](const Camera& p) {
          return p.GetParameterValue(Camera::Parameters::Focal);
        },
        [](Camera& p, double focal) {
          p.SetParameterValue(Camera::Parameters::Focal, focal);
        })
    .def_property(
        "aspect_ratio",
        [](const Camera& p) {
          return p.GetParameterValue(Camera::Parameters::AspectRatio);
        },
        [](Camera& p, double ar) {
          p.SetParameterValue(Camera::Parameters::AspectRatio, ar);
        })
    .def_property(
        "transition",
        [](const Camera& p) {
          return p.GetParameterValue(Camera::Parameters::Transition);
        },
        [](Camera& p, double transition) {
          p.SetParameterValue(Camera::Parameters::Transition, transition);
        })
    .def_property(
        "distortion",
        [](const Camera& p) {
          const auto values_map = p.GetParametersMap();

          std::vector<double> disto_values;
          const auto disto_types = {Camera::Parameters::K1, Camera::Parameters::K2, Camera::Parameters::K3, Camera::Parameters::P1, Camera::Parameters::P2};
          for (const auto type : disto_types) {
            auto find_param = values_map.find(type);
            if (find_param != values_map.end()) {
              disto_values.push_back(find_param->second);
            }
          }
          VecXd distorsion(disto_values.size());
          for (int i = 0; i < disto_values.size(); ++i) {
            distorsion[i] = disto_values[i];
          }
          return distorsion;
        },
        [](Camera& p, const VecXd& distorsion) {
          const auto types = p.GetParametersTypes();
          int count = 0;
          for (int i = 0; i < types.size(); ++i) {
            const int type_int = static_cast<int>(types[i]);
            if (type_int >= static_cast<int>(Camera::Parameters::K1) &&
                type_int <= static_cast<int>(Camera::Parameters::P2)) {
              p.SetParameterValue(types[i], distorsion(count++));
            }
          }
        })
    .def_property(
        "principal_point",
        [](const Camera& p) {
          const auto values_map = p.GetParametersMap();
          return Vec2d(values_map.at(Camera::Parameters::Cx), values_map.at(Camera::Parameters::Cy));
        },
        [](Camera& p, const Vec2d& principal_point) {
          p.SetParameterValue(Camera::Parameters::Cx, principal_point[0]);
          p.SetParameterValue(Camera::Parameters::Cy, principal_point[1]);
        })
    .def_property_readonly("projection_type", &Camera::GetProjectionString)
    .def_property_readonly("k1",[](const Camera& c) {return c.GetParameterValue(Camera::Parameters::K1);})
    .def_property_readonly("k2",[](const Camera& c) {return c.GetParameterValue(Camera::Parameters::K2);})
    .def_property_readonly("k3",[](const Camera& c) {return c.GetParameterValue(Camera::Parameters::K3);})
    .def_property_readonly("p1",[](const Camera& c) {return c.GetParameterValue(Camera::Parameters::P1);})
    .def_property_readonly("p2",[](const Camera& c) {return c.GetParameterValue(Camera::Parameters::P2);})
    .def(py::pickle(
        [](const Camera& p) {
          return py::make_tuple(
              p.GetParametersMap(), p.GetProjectionType(), p.width, p.height, p.id);
        },
        [](py::tuple t) {
          const auto values = t[0].cast<std::map<Camera::Parameters, double>>();
          const auto type = t[1].cast<ProjectionType>();
          const auto width = t[2].cast<int>();
          const auto height = t[3].cast<int>();
          const auto id = t[4].cast<std::string>();
          
          Camera camera = Camera::CreatePerspectiveCamera(0, 0, 0);
          switch (type) {
            case ProjectionType::PERSPECTIVE:{
              camera = Camera::CreatePerspectiveCamera(values.at(Camera::Parameters::Focal),
                                                        values.at(Camera::Parameters::K1),
                                                        values.at(Camera::Parameters::K2));
              break;
            }
            case ProjectionType::BROWN:{
              Vec2d principal_point = Vec2d::Zero();
              principal_point << values.at(Camera::Parameters::Cx), values.at(Camera::Parameters::Cy);
              VecXd distortion(5);
              distortion << values.at(Camera::Parameters::K1), values.at(Camera::Parameters::K2),
                            values.at(Camera::Parameters::K3), values.at(Camera::Parameters::P1),
                            values.at(Camera::Parameters::P2);
              camera = Camera::CreateBrownCamera(values.at(Camera::Parameters::Focal),
                                                  values.at(Camera::Parameters::AspectRatio),
                                                  principal_point, distortion);
              break;
            }
            case ProjectionType::FISHEYE:{
              camera = Camera::CreateFisheyeCamera(values.at(Camera::Parameters::Focal),
                                                    values.at(Camera::Parameters::K1),
                                                    values.at(Camera::Parameters::K2));
              break;
            }
            case ProjectionType::DUAL:{
              camera = Camera::CreateDualCamera(values.at(Camera::Parameters::Transition),
                                                values.at(Camera::Parameters::Focal),
                                                values.at(Camera::Parameters::K1),
                                                values.at(Camera::Parameters::K2));
              break;
            }
            case ProjectionType::SPHERICAL:{
              camera = Camera::CreateSphericalCamera();
              break;
            }
          }
          camera.width = width;
          camera.height = height;
          camera.id = id;
          return camera;
        }))
    // Python2 + copy/deepcopy + pybind11 workaround
    .def("__copy__", [](const Camera& c, const py::dict& d) { return c; }, py::return_value_policy::copy)
    .def("__deepcopy__", [](const Camera& c, const py::dict& d) { return c; }, py::return_value_policy::copy)
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
