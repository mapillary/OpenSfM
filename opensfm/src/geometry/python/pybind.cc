#include <geometry/absolute_pose.h>
#include <geometry/camera.h>
#include <geometry/essential.h>
#include <geometry/pose.h>
#include <geometry/relative_pose.h>
#include <geometry/similarity.h>
#include <geometry/triangulation.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(pygeometry, m) {
  py::enum_<geometry::ProjectionType>(m, "ProjectionType")
      .value("PERSPECTIVE", geometry::ProjectionType::PERSPECTIVE)
      .value("BROWN", geometry::ProjectionType::BROWN)
      .value("FISHEYE", geometry::ProjectionType::FISHEYE)
      .value("FISHEYE_OPENCV", geometry::ProjectionType::FISHEYE_OPENCV)
      .value("FISHEYE62", geometry::ProjectionType::FISHEYE62)
      .value("DUAL", geometry::ProjectionType::DUAL)
      .value("SPHERICAL", geometry::ProjectionType::SPHERICAL)
      .value("RADIAL", geometry::ProjectionType::RADIAL)
      .value("SIMPLE_RADIAL", geometry::ProjectionType::SIMPLE_RADIAL)
      .export_values();

  py::enum_<geometry::Camera::Parameters>(m, "CameraParameters")
      .value("focal", geometry::Camera::Parameters::Focal)
      .value("aspect_ratio", geometry::Camera::Parameters::AspectRatio)
      .value("k1", geometry::Camera::Parameters::K1)
      .value("k2", geometry::Camera::Parameters::K2)
      .value("k3", geometry::Camera::Parameters::K3)
      .value("k4", geometry::Camera::Parameters::K4)
      .value("k5", geometry::Camera::Parameters::K5)
      .value("k6", geometry::Camera::Parameters::K6)
      .value("p1", geometry::Camera::Parameters::P1)
      .value("p2", geometry::Camera::Parameters::P2)
      .value("cx", geometry::Camera::Parameters::Cx)
      .value("cy", geometry::Camera::Parameters::Cy)
      .value("transition", geometry::Camera::Parameters::Transition)
      .value("none", geometry::Camera::Parameters::None)
      .export_values();

  py::class_<geometry::Camera>(m, "Camera")
      .def_static("create_perspective",
                  &geometry::Camera::CreatePerspectiveCamera)
      .def_static("create_brown", &geometry::Camera::CreateBrownCamera)
      .def_static("create_fisheye", &geometry::Camera::CreateFisheyeCamera)
      .def_static("create_fisheye_opencv",
                  &geometry::Camera::CreateFisheyeOpencvCamera)
      .def_static("create_fisheye62", &geometry::Camera::CreateFisheye62Camera)
      .def_static("create_dual", &geometry::Camera::CreateDualCamera)
      .def_static("create_spherical", &geometry::Camera::CreateSphericalCamera)
      .def_static("create_radial", &geometry::Camera::CreateRadialCamera)
      .def_static("create_simple_radial",
                  &geometry::Camera::CreateSimpleRadialCamera)
      .def("pixel_to_normalized_coordinates_common",
           (Vec2d(*)(const Vec2d&, const int, const int)) &
               geometry::Camera::PixelToNormalizedCoordinates)
      .def("normalized_to_pixel_coordinates_common",
           (Vec2d(*)(const Vec2d&, const int, const int)) &
               geometry::Camera::NormalizedToPixelCoordinates)
      .def("project", &geometry::Camera::Project)
      .def("project_many", &geometry::Camera::ProjectMany,
           py::call_guard<py::gil_scoped_release>())
      .def("pixel_bearing", &geometry::Camera::Bearing,
           py::call_guard<py::gil_scoped_release>())
      .def("pixel_bearing_many", &geometry::Camera::BearingsMany,
           py::call_guard<py::gil_scoped_release>())
      .def("get_K", &geometry::Camera::GetProjectionMatrix)
      .def("get_K_in_pixel_coordinates",
           &geometry::Camera::GetProjectionMatrixScaled)
      .def("set_parameter_value", &geometry::Camera::SetParameterValue)
      .def("set_parameters_values", &geometry::Camera::SetParametersValues)
      .def("get_parameters_values", &geometry::Camera::GetParametersValues)
      .def("get_parameters_types", &geometry::Camera::GetParametersTypes)
      .def("get_parameters_map", &geometry::Camera::GetParametersMap)
      .def("pixel_to_normalized_coordinates",
           (Vec2d(geometry::Camera::*)(const Vec2d&) const) &
               geometry::Camera::PixelToNormalizedCoordinates)
      .def("normalized_to_pixel_coordinates",
           (Vec2d(geometry::Camera::*)(const Vec2d&) const) &
               geometry::Camera::NormalizedToPixelCoordinates)
      .def_readwrite("width", &geometry::Camera::width)
      .def_readwrite("height", &geometry::Camera::height)
      .def_readwrite("id", &geometry::Camera::id)
      .def_property(
          "focal",
          [](const geometry::Camera& p) {
            return p.GetParameterValue(geometry::Camera::Parameters::Focal);
          },
          [](geometry::Camera& p, double focal) {
            p.SetParameterValue(geometry::Camera::Parameters::Focal, focal);
          })
      .def_property(
          "aspect_ratio",
          [](const geometry::Camera& p) {
            return p.GetParameterValue(
                geometry::Camera::Parameters::AspectRatio);
          },
          [](geometry::Camera& p, double ar) {
            p.SetParameterValue(geometry::Camera::Parameters::AspectRatio, ar);
          })
      .def_property(
          "transition",
          [](const geometry::Camera& p) {
            return p.GetParameterValue(
                geometry::Camera::Parameters::Transition);
          },
          [](geometry::Camera& p, double transition) {
            p.SetParameterValue(geometry::Camera::Parameters::Transition,
                                transition);
          })
      .def_property(
          "distortion",
          [](const geometry::Camera& p) {
            const auto values_map = p.GetParametersMap();

            std::vector<double> disto_values;
            const auto disto_types = {geometry::Camera::Parameters::K1,
                                      geometry::Camera::Parameters::K2,
                                      geometry::Camera::Parameters::K3,
                                      geometry::Camera::Parameters::K4,
                                      geometry::Camera::Parameters::K5,
                                      geometry::Camera::Parameters::K6,
                                      geometry::Camera::Parameters::P1,
                                      geometry::Camera::Parameters::P2};
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
          [](geometry::Camera& p, const VecXd& distortion) {
            const auto types = p.GetParametersTypes();
            int count = 0;
            for (int i = 0; i < types.size(); ++i) {
              const int type_int = static_cast<int>(types[i]);
              if (type_int >=
                      static_cast<int>(geometry::Camera::Parameters::K1) &&
                  type_int <=
                      static_cast<int>(geometry::Camera::Parameters::P2)) {
                p.SetParameterValue(types[i], distortion(count++));
              }
            }
          })
      .def_property(
          "principal_point",
          [](const geometry::Camera& p) {
            const auto values_map = p.GetParametersMap();
            return Vec2d(values_map.at(geometry::Camera::Parameters::Cx),
                         values_map.at(geometry::Camera::Parameters::Cy));
          },
          [](geometry::Camera& p, const Vec2d& principal_point) {
            p.SetParameterValue(geometry::Camera::Parameters::Cx,
                                principal_point[0]);
            p.SetParameterValue(geometry::Camera::Parameters::Cy,
                                principal_point[1]);
          })
      .def_property_readonly("projection_type",
                             (std::string(geometry::Camera::*)() const) &
                                 geometry::Camera::GetProjectionString)
      .def_static("is_panorama",
                  [](const std::string& s) {
                    return !s.compare("spherical") ||
                           !s.compare("equirectangular");
                  })
      .def_property_readonly(
          "k1",
          [](const geometry::Camera& c) {
            return c.GetParameterValue(geometry::Camera::Parameters::K1);
          })
      .def_property_readonly(
          "k2",
          [](const geometry::Camera& c) {
            return c.GetParameterValue(geometry::Camera::Parameters::K2);
          })
      .def_property_readonly(
          "k3",
          [](const geometry::Camera& c) {
            return c.GetParameterValue(geometry::Camera::Parameters::K3);
          })
      .def_property_readonly(
          "k4",
          [](const geometry::Camera& c) {
            return c.GetParameterValue(geometry::Camera::Parameters::K4);
          })
      .def_property_readonly(
          "k5",
          [](const geometry::Camera& c) {
            return c.GetParameterValue(geometry::Camera::Parameters::K5);
          })
      .def_property_readonly(
          "k6",
          [](const geometry::Camera& c) {
            return c.GetParameterValue(geometry::Camera::Parameters::K6);
          })
      .def_property_readonly(
          "p1",
          [](const geometry::Camera& c) {
            return c.GetParameterValue(geometry::Camera::Parameters::P1);
          })
      .def_property_readonly(
          "p2",
          [](const geometry::Camera& c) {
            return c.GetParameterValue(geometry::Camera::Parameters::P2);
          })
      .def(py::pickle(
          [](const geometry::Camera& p) {
            return py::make_tuple(
                p.GetParametersTypes(), p.GetParametersValues(),
                p.GetProjectionType(), p.width, p.height, p.id);
          },
          [](py::tuple t) {
            const auto types =
                t[0].cast<std::vector<geometry::Camera::Parameters>>();
            const auto values = t[1].cast<VecXd>();
            const auto type = t[2].cast<geometry::ProjectionType>();
            const auto width = t[3].cast<int>();
            const auto height = t[4].cast<int>();
            const auto id = t[5].cast<std::string>();

            geometry::Camera camera = geometry::Camera(type, types, values);
            camera.width = width;
            camera.height = height;
            camera.id = id;
            return camera;
          }))
      // Python2 + copy/deepcopy + pybind11 workaround
      .def(
          "__copy__", [](const geometry::Camera& c) { return c; },
          py::return_value_policy::copy)
      .def(
          "__deepcopy__",
          [](const geometry::Camera& c, const py::dict& d) { return c; },
          py::return_value_policy::copy);
  m.def("compute_camera_mapping", geometry::ComputeCameraMapping,
        py::call_guard<py::gil_scoped_release>());
  m.def("triangulate_bearings_dlt", geometry::TriangulateBearingsDLT,
        py::call_guard<py::gil_scoped_release>());
  m.def("triangulate_bearings_midpoint", geometry::TriangulateBearingsMidpoint,
        py::call_guard<py::gil_scoped_release>());
  m.def("triangulate_two_bearings_midpoint",
        geometry::TriangulateTwoBearingsMidpointSolve<double>,
        py::call_guard<py::gil_scoped_release>());
  m.def("triangulate_two_bearings_midpoint_many",
        geometry::TriangulateTwoBearingsMidpointMany,
        py::call_guard<py::gil_scoped_release>());
  m.def("essential_five_points", geometry::EssentialFivePoints);
  m.def("absolute_pose_three_points", geometry::AbsolutePoseThreePoints);
  m.def("absolute_pose_n_points", geometry::AbsolutePoseNPoints);
  m.def("absolute_pose_n_points_known_rotation",
        geometry::AbsolutePoseNPointsKnownRotation);
  m.def("essential_n_points", geometry::EssentialNPoints);
  m.def("relative_pose_from_essential", geometry::RelativePoseFromEssential);
  m.def("relative_rotation_n_points", geometry::RelativeRotationNPoints);
  m.def("relative_pose_refinement", geometry::RelativePoseRefinement);

  py::class_<geometry::Pose>(m, "Pose")
      .def(py::init<const Mat3d&, const Vec3d&>(),
           py::arg("rotation") = Mat3d::Identity(),
           py::arg("translation") = Vec3d::Zero())
      .def(py::init<const Vec3d&, const Vec3d&>(),
           py::arg("rotation") = Vec3d::Zero(),
           py::arg("translation") = Vec3d::Zero())
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
      .def("inverse", [](const geometry::Pose& p) {
        geometry::Pose new_pose;
        new_pose.SetFromWorldToCamera(p.CameraToWorld());
        return new_pose;
      });

  py::class_<geometry::Similarity>(m, "Similarity")
      .def(py::init<const Vec3d&, const Vec3d&, double>())
      .def_property("translation", &geometry::Similarity::Translation,
                    &geometry::Similarity::SetTranslation)
      .def_property("rotation", &geometry::Similarity::Rotation,
                    &geometry::Similarity::SetRotation)
      .def_property("scale", &geometry::Similarity::Scale,
                    &geometry::Similarity::SetScale)
      .def("transform", &geometry::Similarity::Transform)
      .def("get_rotation_matrix", &geometry::Similarity::RotationMatrix);
}
