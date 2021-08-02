#include <geometry/camera.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <map/map.h>
#include <map/observation.h>

namespace {

class BaseMapFixture : public ::testing::Test {
 public:
  BaseMapFixture() {
    camera = geometry::Camera::CreatePerspectiveCamera(0.5, 0, 0);
    camera.width = 640;
    camera.height = 480;

    rig_camera.id = "rig_camera";
  }
  map::RigCamera rig_camera;
  geometry::Camera camera;
};

class EmptyMapFixture : public BaseMapFixture {
 public:
  map::Map map;
};

TEST_F(EmptyMapFixture, CreateCameraCorrectly) {
  const std::string id = "camera1";
  camera.id = id;
  const auto& my_cam = map.CreateCamera(camera);
  ASSERT_EQ(id, my_cam.id);
}

TEST_F(EmptyMapFixture, ReturnsNumberOfCameras) {
  map.CreateCamera(camera);
  ASSERT_EQ(1, map.NumberOfCameras());
}

TEST_F(EmptyMapFixture, ReturnsAllCameras) {
  map.CreateCamera(camera);
  camera.id = "camera1";
  map.CreateCamera(camera);
  camera.id = "camera2";
  ASSERT_EQ(2, map.GetCameras().size());
}

TEST_F(EmptyMapFixture, CreateLandmarkCorrectly) {
  const std::string id = "0";
  auto& lm = map.CreateLandmark(id, Vec3d::Random());
  ASSERT_EQ(id, lm.id_);
}

TEST_F(EmptyMapFixture, CreateAndReturnLandmarkCorrectly) {
  const std::string id = "0";
  auto& lm = map.CreateLandmark(id, Vec3d::Random());
  ASSERT_EQ(&map.GetLandmark(id), &lm);
}

TEST_F(EmptyMapFixture, HasLandmarkAfterCreation) {
  const std::string id = "0";
  map.CreateLandmark(id, Vec3d::Random());
  ASSERT_TRUE(map.HasLandmark(id));
}

TEST_F(EmptyMapFixture, ThrowOnInvalidCameraAccess) {
  ASSERT_ANY_THROW(map.GetCamera("invalid"));
}

TEST_F(EmptyMapFixture, ThrowOnInvalidLandmarkAccess) {
  ASSERT_ANY_THROW(map.GetLandmark("invalid"));
}

TEST_F(EmptyMapFixture, ThrowOnInvalidShotAccess) {
  ASSERT_ANY_THROW(map.GetShot("invalid"));
}

TEST_F(EmptyMapFixture, CreatePointCorrectly) {
  const int id = 0;
  auto& point = map.CreateLandmark(std::to_string(id), Vec3d::Random());
  ASSERT_EQ(std::to_string(id), point.id_);
}

TEST_F(EmptyMapFixture, CreatePointWithCorrectCoords) {
  const Vec3d position = Vec3d::Random();
  auto& point = map.CreateLandmark(std::to_string(0), position);
  ASSERT_EQ(position, point.GetGlobalPos());
}

TEST_F(EmptyMapFixture, ReturnsHasLandmark) {
  const int id = 0;
  map.CreateLandmark(std::to_string(id), Vec3d::Random());
  ASSERT_TRUE(map.HasLandmark(std::to_string(id)));
}

TEST_F(EmptyMapFixture, CreateRigCameraCorrectly) {
  auto& created_rig_camera = map.CreateRigCamera(rig_camera);
  ASSERT_EQ(created_rig_camera.id, rig_camera.id);
}

class ToyMapFixture : public EmptyMapFixture {
 public:
  ToyMapFixture() {
    int shot_id = 0;
    for (auto i = 0; i < num_cameras; ++i) {
      camera.id = std::to_string(i);
      map.CreateCamera(camera);

      for (auto j = 0; j < num_shots[i]; ++j) {
        map.CreateShot(std::to_string(shot_id++), camera.id);
      }
    }

    for (auto i = 0; i < num_points; ++i) {
      map.CreateLandmark(std::to_string(i), Vec3d::Random());
    }

    map.CreateRigCamera(rig_camera);
  }

  static constexpr int num_points = 10;
  static constexpr int num_cameras = 2;
  std::array<int, 2> num_shots = {5, 3};
};

const int ToyMapFixture::num_points;
const int ToyMapFixture::num_cameras;

TEST_F(ToyMapFixture, ReturnsNumberOfShots) {
  ASSERT_EQ(map.NumberOfShots(), 8);
}

TEST_F(ToyMapFixture, ReturnsNumberOfLandmarks) {
  ASSERT_EQ(map.NumberOfLandmarks(), num_points);
}

TEST_F(ToyMapFixture, RemoveShots) {
  map.RemoveShot("1");
  ASSERT_EQ(map.NumberOfShots(), 7);
}

TEST_F(ToyMapFixture, ThrowsWhenRemovingTwice) {
  map.RemoveShot("1");
  ASSERT_THROW(map.RemoveShot("1"), std::runtime_error);
}

TEST_F(ToyMapFixture, ThrowsWhenCreatingTwice) {
  ASSERT_THROW(map.CreateShot("0", "0"), std::runtime_error);
}

TEST_F(ToyMapFixture, RemoveLandmark) {
  map.RemoveLandmark("1");
  ASSERT_EQ(map.NumberOfLandmarks(), num_points - 1);
}

TEST_F(ToyMapFixture, ThrowsWhenRemovingLandmarkTwice) {
  map.RemoveLandmark("1");
  ASSERT_THROW(map.RemoveLandmark("1"), std::runtime_error);
}

TEST_F(ToyMapFixture, CreateRigInstanceCorrectly) {
  std::map<map::ShotId, map::RigCameraId> instance_shots;
  instance_shots["0"] = rig_camera.id;
  auto& create_rig_instance = map.CreateRigInstance(1, instance_shots);

  ASSERT_EQ(1, create_rig_instance.GetShots().size());
}

TEST_F(ToyMapFixture, UpdatesRigInstanceCorrectly) {
  std::map<map::ShotId, map::RigCameraId> instance_shots;
  instance_shots["0"] = rig_camera.id;

  auto& instance1 = map.CreateRigInstance(1, instance_shots);
  Vec3d rand1 = Vec3d::Random();
  instance1.SetPose(geometry::Pose(rand1));

  auto instance2 = instance1;
  Vec3d rand2 = Vec3d::Random();
  instance2.SetPose(geometry::Pose(rand2));

  map.UpdateRigInstance(instance2);
  ASSERT_EQ(instance1.GetPose().GetOrigin(), instance2.GetPose().GetOrigin());
}

TEST_F(ToyMapFixture, ThrowOnCreateRigInstanceWithInvalidShot) {
  std::map<map::ShotId, map::RigCameraId> instance_shots;
  instance_shots["invalid_shot"] = rig_camera.id;
  ASSERT_THROW(map.CreateRigInstance(1, instance_shots), std::runtime_error);
}

TEST_F(ToyMapFixture, ThrowOnCreateRigInstanceWithInvalidRigCamera) {
  std::map<map::ShotId, map::RigCameraId> instance_shots;
  instance_shots["0"] = "invalid_rig_camera";
  ASSERT_THROW(map.CreateRigInstance(1, instance_shots), std::runtime_error);
}

class OneCameraMapFixture : public EmptyMapFixture {
 public:
  OneCameraMapFixture() {
    camera.id = std::to_string(id);
    map.CreateCamera(camera);
  }
  static int constexpr id = 0;
};

TEST_F(OneCameraMapFixture, ReturnsNumberOfShots) {
  const auto& shot = map.CreateShot("0", "0");
  ASSERT_EQ(shot.id_, "0");
}

TEST_F(OneCameraMapFixture, ReturnsShots) {
  const auto& shot = map.CreateShot("0", "0");
  ASSERT_EQ(&shot, &map.GetShot("0"));
}

TEST_F(OneCameraMapFixture, ReturnsHasShots) {
  map.CreateShot("0", "0");
  ASSERT_TRUE(map.HasShot("0"));
}

TEST_F(EmptyMapFixture, ConstructSmallProblem) {
  constexpr auto n_points{300};
  constexpr auto n_shots{20};
  camera.id = "cam1";
  map.CreateCamera(camera);
  for (auto i = 0; i < n_shots; ++i) {
    const auto shot_id = "shot" + std::to_string(i);
    map.CreateShot(shot_id, camera.id);
  }
  ASSERT_EQ(map.NumberOfShots(), n_shots);
  auto& shots = map.GetShots();
  ASSERT_EQ(shots.size(), n_shots);
  for (auto i = 0; i < n_points; ++i) {
    Eigen::Vector3d pos = Eigen::Vector3d::Random();
    const auto lm_id = std::to_string(i);
    auto& lm = map.CreateLandmark(std::to_string(i), pos);
    // all shots see all landmarks
    map::Observation obs(100, 200, 0.5, 255, 255, 255, i);
    for (auto& shot_pair : shots) {
      map.AddObservation(&shot_pair.second, &lm, obs);
    }
    ASSERT_EQ(lm.NumberOfObservations(), n_shots);
  }
  ASSERT_EQ(map.NumberOfLandmarks(), n_points);
  ASSERT_EQ(map.GetLandmarks().size(), n_points);
}

TEST_F(OneCameraMapFixture, ComputeReprojectionErrorNormalized) {
  const auto& shot = map.CreateShot("0", "0");
  Eigen::Vector3d pos = Eigen::Vector3d::Random();
  map.CreateLandmark("1", pos);

  const Vec2d expected = -shot.Project(pos);
  const auto scale = 0.1;

  auto manager = map::TracksManager();
  const map::Observation o(0., 0., scale, 1, 1, 1, 1, 1, 1);
  manager.AddObservation("0", "1", o);

  auto errors =
      map.ComputeReprojectionErrors(manager, map::Map::ErrorType::Normalized);
  const auto computed = errors["0"]["1"];
  ASSERT_NEAR(expected[0] / scale, computed[0], 1e-8);
  ASSERT_NEAR(expected[1] / scale, computed[1], 1e-8);
}

class OneRigMapFixture : public EmptyMapFixture {
 public:
  OneRigMapFixture() {
    camera.id = "0";
    map.CreateCamera(camera);
    map.CreateShot("0", "0");
    map.CreateRigCamera(rig_camera);
    std::map<map::ShotId, map::RigCameraId> instance_shots;
    instance_shots["0"] = rig_camera.id;
    map.CreateRigInstance(1, instance_shots);
  }
};

TEST_F(OneRigMapFixture, ReturnsNumberOfRigCameras) {
  ASSERT_EQ(1, map.NumberOfRigCameras());
}

TEST_F(OneRigMapFixture, HasRigCameras) {
  ASSERT_TRUE(map.HasRigCamera("rig_camera"));
}

TEST_F(OneRigMapFixture, ReturnsRigCamera) {
  const auto& rig_camera = map.GetRigCamera("rig_camera");
  ASSERT_EQ("rig_camera", rig_camera.id);
}

TEST_F(OneRigMapFixture, ReturnsAllRigCameras) {
  ASSERT_EQ(1, map.GetRigCameras().size());
}

TEST_F(OneRigMapFixture, ReturnsNumberOfRigInstances) {
  ASSERT_EQ(1, map.NumberOfRigInstances());
}

TEST_F(OneRigMapFixture, HasRigInstances) {
  ASSERT_TRUE(map.HasRigInstance(1));
}

TEST_F(OneRigMapFixture, ReturnsRigInstance) {
  const auto& instance = map.GetRigInstance(1);
  std::set<map::ShotId> expected_keys;
  expected_keys.insert("0");
  ASSERT_THAT(instance.GetShotIDs(), ::testing::ContainerEq(expected_keys));
}

TEST_F(OneRigMapFixture, ReturnsAllRigInstances) {
  ASSERT_EQ(1, map.GetRigInstances().size());
}

TEST_F(EmptyMapFixture, TopoCentricConverterAccess) {
  const auto& topo_default = map.GetTopocentricConverter();
  ASSERT_EQ(topo_default.alt_, 0.0);
  ASSERT_EQ(topo_default.lat_, 0.0);
  ASSERT_EQ(topo_default.long_, 0.0);
  map.SetTopocentricConverter(1.0, 2.0, 3.0);
  ASSERT_EQ(topo_default.lat_, 1.0);
  ASSERT_EQ(topo_default.long_, 2.0);
  ASSERT_EQ(topo_default.alt_, 3.0);
  ASSERT_EQ(topo_default.GetLlaRef(), Vec3d(1, 2, 3));
}

TEST_F(ToyMapFixture, ToTracksManager) {
  size_t feat_id = 0;
  for (auto& shot_pair : map.GetShots()) {
    for (auto& lm : map.GetLandmarks()) {
      map.AddObservation(
          &shot_pair.second, &lm.second,
          map::Observation(100, 200, 0.5, 255, 255, 255, feat_id));
    }
    ++feat_id;
  }
  const auto manager = map.ToTracksManager();
  ASSERT_EQ(manager.NumTracks(), map.NumberOfLandmarks());
  ASSERT_EQ(manager.NumShots(), map.NumberOfShots() + map.NumberOfPanoShots());
  for (const auto& shot_pair : map.GetShots()) {
    const auto& shot_obs1 = manager.GetShotObservations(shot_pair.first);
    const auto& shot_obs2 = shot_pair.second.GetLandmarkObservations();

    ASSERT_EQ(shot_obs1.size(), shot_obs2.size());
    for (const auto& lm_obs : shot_obs2) {
      ASSERT_NE(shot_obs1.find(lm_obs.first->id_), shot_obs1.end());
      ASSERT_EQ(lm_obs.second, shot_obs1.at(lm_obs.first->id_));
    }
  }
}

}  // namespace
