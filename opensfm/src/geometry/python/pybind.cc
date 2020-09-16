#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <glog/logging.h>

#include <geometry/essential.h>
#include <geometry/camera.h>
#include <geometry/pose.h>
#include <geometry/relative_pose.h>
#include <geometry/absolute_pose.h>
#include <geometry/triangulation.h>

namespace py = pybind11;

PYBIND11_MODULE(pygeometry, m) {

  py::enum_<ProjectionType>(m, "ProjectionType")
    .value("PERSPECTIVE", ProjectionType::PERSPECTIVE)
    .value("BROWN", ProjectionType::BROWN)
    .value("FISHEYE", ProjectionType::FISHEYE)
    .value("FISHEYE_OPENCV", ProjectionType::FISHEYE_OPENCV)
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
    .value("k4", Camera::Parameters::K4)
    .value("p1", Camera::Parameters::P1)
    .value("p2", Camera::Parameters::P2)
    .value("transition", Camera::Parameters::Transition)
    .export_values()
  ;

  py::class_<Camera>(m, "Camera")
    .def_static("create_perspective", &Camera::CreatePerspectiveCamera)
    .def_static("create_brown", &Camera::CreateBrownCamera)
    .def_static("create_fisheye", &Camera::CreateFisheyeCamera)
    .def_static("create_fisheye_opencv", &Camera::CreateFisheyeExtendedCamera)
    .def_static("create_dual", &Camera::CreateDualCamera)
    .def_static("create_spherical", &Camera::CreateSphericalCamera)
    .def("project", &Camera::Project)
    .def("project_many", &Camera::ProjectMany)
    .def("pixel_bearing", &Camera::Bearing)
    .def("pixel_bearing_many", &Camera::BearingsMany)
    .def("get_K", &Camera::GetProjectionMatrix)
    .def("get_K_in_pixel_coordinates", &Camera::GetProjectionMatrixScaled)
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
          VecXd distortion(disto_values.size());
          for (int i = 0; i < disto_values.size(); ++i) {
            distortion[i] = disto_values[i];
          }
          return distortion;
        },
        [](Camera& p, const VecXd& distortion) {
          const auto types = p.GetParametersTypes();
          int count = 0;
          for (int i = 0; i < types.size(); ++i) {
            const int type_int = static_cast<int>(types[i]);
            if (type_int >= static_cast<int>(Camera::Parameters::K1) &&
                type_int <= static_cast<int>(Camera::Parameters::P2)) {
              p.SetParameterValue(types[i], distortion(count++));
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
    .def_static("is_panorama", [](const std::string& s) {return !s.compare("spherical") || !s.compare("equirectangular");})
    .def_property_readonly("k1",[](const Camera& c) {return c.GetParameterValue(Camera::Parameters::K1);})
    .def_property_readonly("k2",[](const Camera& c) {return c.GetParameterValue(Camera::Parameters::K2);})
    .def_property_readonly("k3",[](const Camera& c) {return c.GetParameterValue(Camera::Parameters::K3);})
    .def_property_readonly("k4",[](const Camera& c) {return c.GetParameterValue(Camera::Parameters::K4);})
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
            case ProjectionType::FISHEYE_OPENCV:{
              Vec2d principal_point = Vec2d::Zero();
              principal_point << values.at(Camera::Parameters::Cx), values.at(Camera::Parameters::Cy);
              VecXd distortion(4);
              distortion << values.at(Camera::Parameters::K1), values.at(Camera::Parameters::K2),
                            values.at(Camera::Parameters::K3), values.at(Camera::Parameters::K4);
              camera = Camera::CreateFisheyeExtendedCamera(values.at(Camera::Parameters::Focal),
                                                           values.at(Camera::Parameters::AspectRatio),
                                                           principal_point, distortion);
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
    .def("__copy__", [](const Camera& c) { return c; }, py::return_value_policy::copy)
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

  py::class_<geometry::Pose>(m, "Pose")
      .def(py::init<const Mat3d&, const Vec3d&>(),
           py::arg("rotation") = Mat3d::Identity(), py::arg("translation") = Vec3d::Zero())
      .def(py::init<const Vec3d&, const Vec3d&>(),
          py::arg("rotation") = Vec3d::Zero(), py::arg("translation") = Vec3d::Zero())
      .def(py::init<const Vec3d&>())
      .def("get_cam_to_world", &geometry::Pose::CameraToWorld)
      .def("get_world_to_cam", &geometry::Pose::WorldToCamera)
      // C++11
      .def("set_from_world_to_cam", (void (geometry::Pose::*)(const Mat4d&)) &
                                        geometry::Pose::SetFromWorldToCamera)
      .def("set_from_world_to_cam",
           (void (geometry::Pose::*)(const Mat3d&, const Vec3d&)) &
               geometry::Pose::SetFromWorldToCamera)
      .def("set_from_world_to_cam",
           (void (geometry::Pose::*)(const Vec3d&, const Vec3d&)) &
               geometry::Pose::SetFromWorldToCamera)
      .def("set_from_cam_to_world", (void (geometry::Pose::*)(const Mat4d&)) &
                                        geometry::Pose::SetFromCameraToWorld)
      .def("set_from_cam_to_world",
           (void (geometry::Pose::*)(const Mat3d&, const Vec3d&)) &
               geometry::Pose::SetFromCameraToWorld)
      .def("set_from_cam_to_world",
           (void (geometry::Pose::*)(const Vec3d&, const Vec3d&)) &
               geometry::Pose::SetFromCameraToWorld)
      .def("get_origin", &geometry::Pose::GetOrigin)
      .def("set_origin", &geometry::Pose::SetOrigin)
      .def("get_R_cam_to_world", &geometry::Pose::RotationCameraToWorld)
      .def("get_rotation_matrix", &geometry::Pose::RotationWorldToCamera)
      .def("get_R_world_to_cam", &geometry::Pose::RotationWorldToCamera)
      .def("get_R_cam_to_world_min", &geometry::Pose::RotationCameraToWorldMin)
      .def("get_R_world_to_cam_min", &geometry::Pose::RotationWorldToCameraMin)
      .def("get_t_cam_to_world", &geometry::Pose::TranslationCameraToWorld)
      .def("get_t_world_to_cam", &geometry::Pose::TranslationWorldToCamera)
      .def("get_Rt", &geometry::Pose::WorldToCameraRt)
      .def_property("rotation", &geometry::Pose::RotationWorldToCameraMin,
                    &geometry::Pose::SetWorldToCamRotation)
      .def_property("translation", &geometry::Pose::TranslationWorldToCamera,
                    &geometry::Pose::SetWorldToCamTranslation)
      .def("set_rotation_matrix", &geometry::Pose::SetWorldToCamRotationMatrix)
      .def("transform", &geometry::Pose::TransformWorldToCamera)
      .def("transform_inverse", &geometry::Pose::TransformCameraToWorld)
      .def("transform_many", &geometry::Pose::TransformWorldToCameraMany)
      .def("transform_inverse_many",
           &geometry::Pose::TransformCameraToWorldMany)
      .def("relative_to", &geometry::Pose::RelativeTo)
      .def("compose", &geometry::Pose::Compose)
      .def(py::pickle(
          [](const geometry::Pose& p) {
            return py::make_tuple(p.CameraToWorld());
          },
          [](py::tuple p) {
            geometry::Pose pose;
            pose.SetFromCameraToWorld(p[0].cast<Mat4d>());
            return pose;
          }))
      .def(
          "__copy__", [](const geometry::Pose& p) { return p; },
          py::return_value_policy::copy)
      .def(
          "__deepcopy__",
          [](const geometry::Pose& p, const py::dict& d) { return p; },
          py::return_value_policy::copy)
      .def("inverse", [](const geometry::Pose& p){
        geometry::Pose new_pose;
        new_pose.SetFromWorldToCamera(p.CameraToWorld());
        return new_pose;
      })
      ;
}
