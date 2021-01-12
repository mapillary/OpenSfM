#include <foundation/types.h>
#include <geometry/camera.h>
#include <geometry/pose.h>
#include <map/ba_helpers.h>
#include <map/dataviews.h>
#include <map/defines.h>
#include <map/ground_control_points.h>
#include <map/landmark.h>
#include <map/map.h>
#include <map/pybind_utils.h>
#include <map/shot.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <iostream>
#include <typeinfo>
namespace py = pybind11;

template <typename T>
void DeclareShotMeasurement(py::module &m, const std::string &type_name) {
  using SM = map::ShotMeasurement<T>;

  std::string class_name = std::string("ShotMeasurement") + type_name;

  py::class_<SM>(m, class_name.c_str())
      .def(py::init<>())
      .def_property_readonly("has_value", &SM::HasValue)
      .def_property("value", &SM::Value, &SM::SetValue)
      .def("reset", &SM::Reset)
      .def(py::pickle(
          [](const SM &sm) {
            return py::make_tuple(sm.HasValue(), sm.Value());
          },
          [](py::tuple p) {
            SM sm;
            const auto has_value = p[0].cast<bool>();
            if (has_value) {
              sm.SetValue(p[1].cast<T>());
            }
            return sm;
          }));
}
PYBIND11_MODULE(pymap, m) {
  py::class_<map::Map>(m, "Map")
      .def(py::init())
      .def("number_of_shots", &map::Map::NumberOfShots,
           "Returns the number of shots")
      .def("number_of_pano_shots", &map::Map::NumberOfPanoShots)
      .def("number_of_landmarks", &map::Map::NumberOfLandmarks)
      .def("number_of_cameras", &map::Map::NumberOfCameras)
      .def("create_camera", &map::Map::CreateCamera, py::arg("camera"),
           py::return_value_policy::reference_internal)
      // Landmark
      .def("create_landmark", &map::Map::CreateLandmark, py::arg("lm_id"),
           py::arg("global_position"),
           py::return_value_policy::reference_internal)
      .def("remove_landmark", (void (map::Map::*)(const map::Landmark *const)) &
                                  map::Map::RemoveLandmark)
      .def("remove_landmark", (void (map::Map::*)(const map::LandmarkId &)) &
                                  map::Map::RemoveLandmark)
      .def(
          "create_shot",
          (map::Shot * (map::Map::*)(const map::ShotId &, const map::CameraId &,
                                     const geometry::Pose &)) &
              map::Map::CreateShot,
          py::arg("shot_id"), py::arg("camera_id"), py::arg("pose"),
          py::return_value_policy::reference_internal)
      .def("create_shot",
           (map::Shot *
            (map::Map::*)(const map::ShotId &, const map::CameraId &)) &
               map::Map::CreateShot,
           py::arg("shot_id"), py::arg("camera_id"),
           py::return_value_policy::reference_internal)
      .def(
          "create_pano_shot",
          (map::Shot * (map::Map::*)(const map::ShotId &, const map::CameraId &,
                                     const geometry::Pose &)) &
              map::Map::CreatePanoShot,
          py::return_value_policy::reference_internal)
      .def("create_pano_shot",
           (map::Shot *
            (map::Map::*)(const map::ShotId &, const map::CameraId &)) &
               map::Map::CreatePanoShot,
           py::return_value_policy::reference_internal)
      .def("remove_shot", &map::Map::RemoveShot)
      .def("get_shot", &map::Map::GetShot,
           py::return_value_policy::reference_internal)
      .def("remove_pano_shot", &map::Map::RemovePanoShot)
      .def("get_pano_shot", &map::Map::GetPanoShot,
           py::return_value_policy::reference_internal)

      .def("add_observation",
           (void (map::Map::*)(map::Shot *const, map::Landmark *const,
                               const map::FeatureId)) &
               map::Map::AddObservation,
           py::arg("shot"), py::arg("landmark"), py::arg("feature_id"))
      .def("add_observation",
           (void (map::Map::*)(map::Shot *const, map::Landmark *const,
                               const Observation &)) &
               map::Map::AddObservation,
           py::arg("shot"), py::arg("landmark"), py::arg("observation"))
      .def("add_observation",
           (void (map::Map::*)(const map::ShotId &, const map::LandmarkId &,
                               const Observation &)) &
               map::Map::AddObservation,
           py::arg("shot_Id"), py::arg("landmark_id"), py::arg("observation"))
      .def("remove_observation",
           (void (map::Map::*)(map::Shot *const, map::Landmark *const,
                               const map::FeatureId)) &
               map::Map::RemoveObservation,
           py::arg("shot"), py::arg("landmark"), py::arg("feature_id"))
      .def("remove_observation",
           (void (map::Map::*)(const map::ShotId &, const map::LandmarkId &)) &
               map::Map::RemoveObservation,
           py::arg("shot"), py::arg("landmark"))

      .def("get_shots", &map::Map::GetShotView)
      .def("get_pano_shots", &map::Map::GetPanoShotView)
      .def("get_cameras", &map::Map::GetCameraView)
      .def("get_camera_view", &map::Map::GetCameraView)
      .def("get_landmarks", &map::Map::GetLandmarkView)
      .def("get_landmark_view", &map::Map::GetLandmarkView)
      .def("set_reference", &map::Map::SetTopocentricConverter)
      .def("get_reference",
           [](const map::Map &map) {
             py::module::import("opensfm.pygeo");
             return map.GetTopocentricConverter();
           })
      .def("create_camera", &map::Map::CreateCamera,
           py::return_value_policy::reference_internal)
      .def("has_landmark", &map::Map::HasLandmark)
      .def("get_landmark", &map::Map::GetLandmark,
           py::return_value_policy::reference_internal)
      .def("clear_observations_and_landmarks",
           &map::Map::ClearObservationsAndLandmarks)
      .def("get_camera", &map::Map::GetCamera,
           py::return_value_policy::reference_internal)
      .def("update_shot", &map::Map::UpdateShot,
           py::return_value_policy::reference_internal)
      .def("update_pano_shot", &map::Map::UpdatePanoShot,
           py::return_value_policy::reference_internal);

  py::class_<map::Shot>(m, "Shot")
      .def_readonly("id", &map::Shot::id_)
      .def_readonly("unique_id", &map::Shot::unique_id_)
      .def_readonly("slam_data", &map::Shot::slam_data_,
                    py::return_value_policy::reference_internal)
      .def_readwrite("mesh", &map::Shot::mesh)
      .def_property("covariance", &map::Shot::GetCovariance,
                    &map::Shot::SetCovariance)
      .def_readwrite("merge_cc", &map::Shot::merge_cc)
      .def_readwrite("scale", &map::Shot::scale)
      .def("get_observation", &map::Shot::GetObservation,
           py::return_value_policy::reference_internal)
      .def("get_keypoints", &map::Shot::GetKeyPoints,
           py::return_value_policy::reference_internal)
      .def("compute_num_valid_pts", &map::Shot::ComputeNumValidLandmarks,
           py::arg("min_obs_thr") = 1)
      .def("get_valid_landmarks", &map::Shot::ComputeValidLandmarks,
           py::return_value_policy::reference_internal)
      .def("init_and_take_datastructures",
           &map::Shot::InitAndTakeDatastructures)
      .def("init_keypts_and_descriptors", &map::Shot::InitKeyptsAndDescriptors)
      .def("set_pose", &map::Shot::SetPose)
      .def("get_pose",
           (const geometry::Pose &(map::Shot::*)() const) & map::Shot::GetPose,
           py::return_value_policy::reference_internal)
      .def("compute_median_depth", &map::Shot::ComputeMedianDepthOfLandmarks)
      .def("scale_landmarks", &map::Shot::ScaleLandmarks)
      .def("scale_pose", &map::Shot::ScalePose)
      .def("remove_observation", &map::Shot::RemoveLandmarkObservation)
      .def("get_camera_to_world", &map::Shot::GetCamToWorld)
      .def("get_world_to_camera", &map::Shot::GetWorldToCam)
      .def("get_obs_by_idx", &map::Shot::GetKeyPointEigen)
      .def("get_camera_name", &map::Shot::GetCameraName)
      .def_property("metadata", &map::Shot::GetShotMeasurements,
                    &map::Shot::SetShotMeasurements,
                    py::return_value_policy::reference_internal)
      .def_property(
          "pose",
          (const geometry::Pose &(map::Shot::*)() const) & map::Shot::GetPose,
          &map::Shot::SetPose)
      .def_property_readonly("camera", &map::Shot::GetCamera,
                             py::return_value_policy::reference_internal)
      .def("create_observation", &map::Shot::CreateObservation)
      .def("get_landmark_observation", &map::Shot::GetLandmarkObservation,
           py::return_value_policy::reference_internal)
      .def("project", &map::Shot::Project)
      .def("project_many", &map::Shot::ProjectMany)
      .def("bearing", &map::Shot::Bearing)
      .def("bearing_many", &map::Shot::BearingMany)
      // pickle support
      .def(py::pickle(
          [](const map::Shot &s) {
            auto c = s.GetCamera();
            return py::make_tuple(
                s.id_, s.unique_id_, s.GetPose().CameraToWorld(),
                py::make_tuple(c->GetParametersMap(), c->GetProjectionType(),
                               c->width, c->height, c->id));
          },
          [](py::tuple s) {
            // Create camera
            auto t = s[3].cast<py::tuple>();
            const auto values =
                t[0].cast<std::map<Camera::Parameters, double>>();
            const auto type = t[1].cast<ProjectionType>();
            const auto width = t[2].cast<int>();
            const auto height = t[3].cast<int>();
            const auto id = t[4].cast<std::string>();

            Camera camera = Camera::CreatePerspectiveCamera(0, 0, 0);
            switch (type) {
              case ProjectionType::PERSPECTIVE: {
                camera = Camera::CreatePerspectiveCamera(
                    values.at(Camera::Parameters::Focal),
                    values.at(Camera::Parameters::K1),
                    values.at(Camera::Parameters::K2));
                break;
              }
              case ProjectionType::BROWN: {
                Vec2d principal_point = Vec2d::Zero();
                principal_point << values.at(Camera::Parameters::Cx),
                    values.at(Camera::Parameters::Cy);
                VecXd distortion(5);
                distortion << values.at(Camera::Parameters::K1),
                    values.at(Camera::Parameters::K2),
                    values.at(Camera::Parameters::K3),
                    values.at(Camera::Parameters::P1),
                    values.at(Camera::Parameters::P2);
                camera = Camera::CreateBrownCamera(
                    values.at(Camera::Parameters::Focal),
                    values.at(Camera::Parameters::AspectRatio), principal_point,
                    distortion);
                break;
              }
              case ProjectionType::FISHEYE: {
                camera = Camera::CreateFisheyeCamera(
                    values.at(Camera::Parameters::Focal),
                    values.at(Camera::Parameters::K1),
                    values.at(Camera::Parameters::K2));
                break;
              }
              case ProjectionType::FISHEYE_OPENCV: {
                Vec2d principal_point = Vec2d::Zero();
                principal_point << values.at(Camera::Parameters::Cx),
                    values.at(Camera::Parameters::Cy);
                VecXd distortion(4);
                distortion << values.at(Camera::Parameters::K1),
                    values.at(Camera::Parameters::K2),
                    values.at(Camera::Parameters::K3),
                    values.at(Camera::Parameters::K4);
                camera = Camera::CreateFisheyeOpencvCamera(
                    values.at(Camera::Parameters::Focal),
                    values.at(Camera::Parameters::AspectRatio), principal_point,
                    distortion);
                break;
              }
              case ProjectionType::FISHEYE62: {
                Vec2d principal_point = Vec2d::Zero();
                principal_point << values.at(Camera::Parameters::Cx),
                    values.at(Camera::Parameters::Cy);
                VecXd distortion(8);
                distortion << values.at(Camera::Parameters::K1),
                    values.at(Camera::Parameters::K2),
                    values.at(Camera::Parameters::K3),
                    values.at(Camera::Parameters::K4),
                    values.at(Camera::Parameters::K5),
                    values.at(Camera::Parameters::K6),
                    values.at(Camera::Parameters::P1),
                    values.at(Camera::Parameters::P2);
                camera = Camera::CreateFisheye62Camera(
                    values.at(Camera::Parameters::Focal),
                    values.at(Camera::Parameters::AspectRatio), principal_point,
                    distortion);
                break;
              }
              case ProjectionType::RADIAL: {
                const Vec2d principal_point(values.at(Camera::Parameters::Cx),
                                            values.at(Camera::Parameters::Cy));
                const Vec2d distortion(values.at(Camera::Parameters::K1),
                                       values.at(Camera::Parameters::K2));
                camera = Camera::CreateRadialCamera(
                    values.at(Camera::Parameters::Focal),
                    values.at(Camera::Parameters::AspectRatio), principal_point,
                    distortion);
                break;
              }
              case ProjectionType::SIMPLE_RADIAL: {
                const Vec2d principal_point(values.at(Camera::Parameters::Cx),
                                            values.at(Camera::Parameters::Cy));
                camera = Camera::CreateSimpleRadialCamera(
                    values.at(Camera::Parameters::Focal),
                    values.at(Camera::Parameters::AspectRatio), principal_point,
                    values.at(Camera::Parameters::K1));
                break;
              }
              case ProjectionType::DUAL: {
                camera = Camera::CreateDualCamera(
                    values.at(Camera::Parameters::Transition),
                    values.at(Camera::Parameters::Focal),
                    values.at(Camera::Parameters::K1),
                    values.at(Camera::Parameters::K2));
                break;
              }
              case ProjectionType::SPHERICAL: {
                camera = Camera::CreateSphericalCamera();
                break;
              }
            }
            camera.width = width;
            camera.height = height;
            camera.id = id;
            // create unique_ptr
            auto cam_ptr = std::unique_ptr<Camera>(new Camera(camera));
            auto pose = geometry::Pose();
            pose.SetFromCameraToWorld(s[2].cast<Mat4d>());
            auto shot =
                map::Shot(s[0].cast<map::ShotId>(), std::move(cam_ptr), pose);
            shot.unique_id_ = s[1].cast<map::ShotUniqueId>();
            return shot;
          }));

  py::class_<map::SLAMShotData>(m, "SlamShotData")
      .def_readonly("undist_keypts", &map::SLAMShotData::undist_keypts_,
                    py::return_value_policy::reference_internal);

  py::class_<map::SLAMLandmarkData>(m, "SlamLandmarkData")
      .def("get_observed_ratio", &map::SLAMLandmarkData::GetObservedRatio);

  DeclareShotMeasurement<int>(m, "Int");
  DeclareShotMeasurement<double>(m, "Double");
  DeclareShotMeasurement<Vec3d>(m, "Vec3d");
  DeclareShotMeasurement<std::string>(m, "String");

  py::class_<map::ShotMeasurements>(m, "ShotMeasurements")
      .def(py::init<>())
      .def_readwrite("gps_accuracy", &map::ShotMeasurements::gps_accuracy_)
      .def_readwrite("gps_position", &map::ShotMeasurements::gps_position_)
      .def_readwrite("orientation", &map::ShotMeasurements::orientation_)
      .def_readwrite("capture_time", &map::ShotMeasurements::capture_time_)
      .def_readwrite("accelerometer", &map::ShotMeasurements::accelerometer_)
      .def_readwrite("compass_angle", &map::ShotMeasurements::compass_angle_)
      .def_readwrite("compass_accuracy",
                     &map::ShotMeasurements::compass_accuracy_)
      .def_readwrite("sequence_key", &map::ShotMeasurements::sequence_key_)
      .def(py::pickle(
          [](const map::ShotMeasurements &s) {
            return py::make_tuple(s.gps_accuracy_, s.gps_position_,
                                  s.orientation_, s.capture_time_,
                                  s.accelerometer_, s.compass_angle_,
                                  s.compass_accuracy_, s.sequence_key_);
          },
          [](py::tuple s) {
            map::ShotMeasurements sm;
            sm.gps_accuracy_ = s[0].cast<decltype(sm.gps_accuracy_)>();
            sm.gps_position_ = s[1].cast<decltype(sm.gps_position_)>();
            sm.orientation_ = s[2].cast<decltype(sm.orientation_)>();
            sm.capture_time_ = s[3].cast<decltype(sm.capture_time_)>();
            sm.accelerometer_ = s[4].cast<decltype(sm.accelerometer_)>();
            sm.compass_angle_ = s[5].cast<decltype(sm.compass_angle_)>();
            sm.compass_accuracy_ = s[5].cast<decltype(sm.compass_angle_)>();
            sm.sequence_key_ = s[6].cast<decltype(sm.sequence_key_)>();
            return sm;
          }))
      .def("__copy__",
           [](const map::ShotMeasurements &to_copy) {
             map::ShotMeasurements copy;
             copy.Set(to_copy);
             return copy;
           },
           py::return_value_policy::copy)

      .def("set", &map::ShotMeasurements::Set);

  py::class_<map::ShotMesh>(m, "ShotMesh")
      .def_property("faces", &map::ShotMesh::GetFaces, &map::ShotMesh::SetFaces)
      .def_property("vertices", &map::ShotMesh::GetVertices,
                    &map::ShotMesh::SetVertices);

  py::class_<map::Landmark>(m, "Landmark")
      .def(py::init<const map::LandmarkId &, const Vec3d &>())
      .def_readonly("id", &map::Landmark::id_)
      .def_readonly("unique_id", &map::Landmark::unique_id_)
      .def_readwrite("slam_data", &map::Landmark::slam_data_)
      .def("get_global_pos", &map::Landmark::GetGlobalPos)
      .def_property("coordinates", &map::Landmark::GetGlobalPos,
                    &map::Landmark::SetGlobalPos)
      .def("set_global_pos", &map::Landmark::SetGlobalPos)
      .def("is_observed_in_shot", &map::Landmark::IsObservedInShot)
      .def("add_observation", &map::Landmark::AddObservation)
      .def("remove_observation", &map::Landmark::RemoveObservation)
      .def("has_observations", &map::Landmark::HasObservations)
      .def("get_observations", &map::Landmark::GetObservations,
           py::return_value_policy::reference_internal)
      .def("number_of_observations", &map::Landmark::NumberOfObservations)
      .def("get_ref_shot", &map::Landmark::GetRefShot,
           py::return_value_policy::reference_internal)
      .def("set_ref_shot", &map::Landmark::SetRefShot)
      .def("get_obs_in_shot", &map::Landmark::GetObservationInShot)
      .def_property("reprojection_errors",
                    &map::Landmark::GetReprojectionErrors,
                    &map::Landmark::SetReprojectionErrors)
      .def("remove_reprojection_error", &map::Landmark::RemoveReprojectionError)
      .def_property("color", &map::Landmark::GetColor,
                    &map::Landmark::SetColor);

  py::class_<map::PanoShotView>(m, "PanoShotView")
      .def(py::init<map::Map &>())
      .def("__len__", &map::PanoShotView::NumberOfShots)
      .def("items",
           [](const map::PanoShotView &sv) {
             auto &shots = sv.GetShots();
             return py::make_ref_iterator(shots.begin(), shots.end());
           },
           py::return_value_policy::reference_internal)
      .def("values",
           [](const map::PanoShotView &sv) {
             auto &shots = sv.GetShots();
             return py::make_ref_value_iterator(shots.begin(), shots.end());
           },
           py::return_value_policy::reference_internal)
      .def("__iter__",
           [](const map::PanoShotView &sv) {
             const auto &shots = sv.GetShots();
             return py::make_key_iterator(shots.begin(), shots.end());
           },
           py::return_value_policy::reference_internal)
      .def("keys",
           [](const map::PanoShotView &sv) {
             const auto &shots = sv.GetShots();
             return py::make_key_iterator(shots.begin(), shots.end());
           },
           py::return_value_policy::reference_internal)
      .def("get", &map::PanoShotView::GetShot,
           py::return_value_policy::reference_internal)
      .def("__getitem__", &map::PanoShotView::GetShot,
           py::return_value_policy::reference_internal)
      .def("__contains__", &map::PanoShotView::HasShot);

  py::class_<map::ShotView>(m, "ShotView")
      .def(py::init<map::Map &>())
      .def("__len__", &map::ShotView::NumberOfShots)
      .def("items",
           [](const map::ShotView &sv) {
             const auto &shots = sv.GetShots();
             return py::make_ref_iterator(shots.begin(), shots.end());
           },
           py::return_value_policy::reference_internal)
      .def("values",
           [](const map::ShotView &sv) {
             const auto &shots = sv.GetShots();
             return py::make_ref_value_iterator(shots.begin(), shots.end());
           },
           py::return_value_policy::reference_internal)
      .def("__iter__",
           [](const map::ShotView &sv) {
             const auto &shots = sv.GetShots();
             return py::make_key_iterator(shots.begin(), shots.end());
           },
           py::return_value_policy::reference_internal)
      .def("keys",
           [](const map::ShotView &sv) {
             const auto &shots = sv.GetShots();
             return py::make_key_iterator(shots.begin(), shots.end());
           },
           py::return_value_policy::reference_internal)
      .def("get", &map::ShotView::GetShot,
           py::return_value_policy::reference_internal)
      .def("__getitem__", &map::ShotView::GetShot,
           py::return_value_policy::reference_internal)
      .def("__contains__", &map::ShotView::HasShot);

  py::class_<map::LandmarkView>(m, "LandmarkView")
      .def(py::init<map::Map &>())
      .def("__len__", &map::LandmarkView::NumberOfLandmarks)
      .def("items",
           [](const map::LandmarkView &sv) {
             auto &lms = sv.GetLandmarks();
             return py::make_ref_iterator(lms.begin(), lms.end());
           },
           py::return_value_policy::reference_internal)
      .def("values",
           [](const map::LandmarkView &sv) {
             auto &lms = sv.GetLandmarks();
             return py::make_ref_value_iterator(lms.begin(), lms.end());
           },
           py::return_value_policy::reference_internal)
      .def("__iter__",
           [](const map::LandmarkView &sv) {
             const auto &lms = sv.GetLandmarks();
             return py::make_key_iterator(lms.begin(), lms.end());
           },
           py::return_value_policy::reference_internal)
      .def("keys",
           [](const map::LandmarkView &sv) {
             const auto &lms = sv.GetLandmarks();
             return py::make_key_iterator(lms.begin(), lms.end());
           },
           py::return_value_policy::reference_internal)
      // Python 2
      // .def(
      //     "__iteritems__",
      //     [](const map::LandmarkView &sv) {
      //       auto &lms = sv.GetLandmarks();
      //       return py::make_ref_iterator(lms.begin(), lms.end());
      //     },
      //     py::return_value_policy::reference_internal)
      // .def(
      //     "iterkeys",
      //     [](const map::LandmarkView &sv) {
      //       const auto &lms = sv.GetLandmarks();
      //       return py::make_key_iterator(lms.begin(), lms.end());
      //     },
      //     py::return_value_policy::reference_internal)
      // .def(
      //     "itervalues",
      //     [](const map::LandmarkView &sv) {
      //       auto &lms = sv.GetLandmarks();
      //       return py::make_ref_value_iterator(lms.begin(), lms.end());
      //     },
      //     py::return_value_policy::reference_internal)
      .def("get", &map::LandmarkView::GetLandmark,
           py::return_value_policy::reference_internal)
      .def("__getitem__", &map::LandmarkView::GetLandmark,
           py::return_value_policy::reference_internal)
      .def("__contains__", &map::LandmarkView::HasLandmark);
  py::class_<map::CameraView>(m, "CameraView")
      .def(py::init<map::Map &>())
      .def("__len__", &map::CameraView::NumberOfCameras)
      .def("items",
           [](const map::CameraView &sv) {
             const auto &cams = sv.GetCameras();
             return py::make_iterator(cams.begin(), cams.end());
           },
           py::return_value_policy::reference_internal)
      .def("values",
           [](map::CameraView &sv) {
             auto &cams = sv.GetCameras();
             return py::make_ref_value_iterator(cams.begin(), cams.end());
           },
           py::return_value_policy::reference_internal)
      .def("__iter__",
           [](const map::CameraView &sv) {
             const auto &cams = sv.GetCameras();
             return py::make_key_iterator(cams.begin(), cams.end());
           },
           py::return_value_policy::reference_internal)
      .def("keys",
           [](const map::CameraView &sv) {
             const auto &cams = sv.GetCameras();
             return py::make_key_iterator(cams.begin(), cams.end());
           },
           py::return_value_policy::reference_internal)
      .def("get", &map::CameraView::GetCamera,
           py::return_value_policy::reference_internal)
      .def("__getitem__", &map::CameraView::GetCamera,
           py::return_value_policy::reference_internal)
      .def("__contains__", &map::CameraView::HasCamera);

  py::class_<BAHelpers>(m, "BAHelpers")
      .def("bundle", &BAHelpers::Bundle)
      .def("bundle_local", &BAHelpers::BundleLocal)
      .def("shot_neighborhood_ids", &BAHelpers::ShotNeighborhoodIds)
      .def("detect_alignment_constraints",
           &BAHelpers::DetectAlignmentConstraints);

  py::class_<map::GroundControlPointObservation>(
      m, "GroundControlPointObservation")
      .def(py::init())
      .def(py::init<const map::ShotId &, const Vec2d &>())
      .def_readwrite("shot_id", &map::GroundControlPointObservation::shot_id_)
      .def_readwrite("projection",
                     &map::GroundControlPointObservation::projection_);
  py::class_<map::GroundControlPoint>(m, "GroundControlPoint")
      .def(py::init())
      .def_readwrite("id", &map::GroundControlPoint::id_)
      .def_readwrite("coordinates", &map::GroundControlPoint::coordinates_)
      .def_readwrite("has_altitude", &map::GroundControlPoint::has_altitude_)
      .def_readwrite("lla", &map::GroundControlPoint::lla_)
      .def_property("observations", &map::GroundControlPoint::GetObservations,
                    &map::GroundControlPoint::SetObservations)
      .def("add_observation", &map::GroundControlPoint::AddObservation);
}
