#include <map/map.h>
#include <geometry/camera.h>
#include <sfm/observation.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>


namespace {

static map::Map CreateDefaultMap(const size_t n_cams, const std::map<map::CameraId, size_t>& cam_shots = {}, const size_t n_points = 0)
{
  Camera camera = Camera::CreatePerspectiveCamera(0.5, 0, 0);
  camera.width = 640;
  camera.height = 480;
  auto map = map::Map();
  for (auto i = 0; i < n_cams; ++i)
  {
    camera.id = std::to_string(i);
    auto* my_cam = map.CreateCamera(camera);
  }
  if (n_cams > 0)
  {
    for (const auto& cam_shot : cam_shots)
    {
      for (auto i = 0; i < cam_shot.second; ++i)
      {
        map.CreateShot(std::to_string(i), cam_shot.first);
      }
    }
  }

  for (auto i = 0; i < n_points; ++i)
  {
    map.CreateLandmark(std::to_string(i), Vec3d::Random());
  }
  return map;
}


TEST(Map, TestCreateCamera) {
  Camera camera = Camera::CreatePerspectiveCamera(0.5, 0, 0);
  camera.width = 640;
  camera.height = 480;
  auto map = map::Map();
  constexpr auto n_cameras{10};
  for (auto i = 0; i < n_cameras; ++i)
  {
    camera.id = std::to_string(i);
    auto* my_cam = map.CreateCamera(camera);
    ASSERT_EQ(my_cam->id, camera.id);
    ASSERT_EQ(my_cam->width, camera.width);
    ASSERT_EQ(my_cam->height, camera.height);
  }
  ASSERT_EQ(map.NumberOfCameras(), n_cameras);
  ASSERT_EQ(map.GetAllCameras().size(), n_cameras);
}

TEST(Map, TestDeleteShots)
{
  std::map<map::CameraId, size_t> cam_shots{{"0", 3}};
  auto map = CreateDefaultMap(1, cam_shots);
  ASSERT_TRUE(map.NumberOfShots() == 3) << map.NumberOfShots() << "== " << 3;
  map.RemoveShot("1");
  ASSERT_TRUE(map.NumberOfShots() == 2) << map.NumberOfShots() << "== " << 2;
  map.RemoveShot("1");
  ASSERT_TRUE(map.NumberOfShots() == 2) << map.NumberOfShots() << "== " << 2;
  map.RemoveShot("2");
  ASSERT_TRUE(map.NumberOfShots() == 1) << map.NumberOfShots() << "== " << 1;
  map.RemoveShot("0");
  ASSERT_TRUE(map.NumberOfShots() == 0) << map.NumberOfShots() << "== " << 0;
}

TEST(Map, TestCreateShots)
{
  auto map = CreateDefaultMap(1);
  auto* shot1 = map.CreateShot("0", "0");
  auto* shot1_1 = map.CreateShot("0", "0");
  ASSERT_EQ(shot1, shot1_1);
  auto* shot2 = map.CreateShot("1", "0");
  ASSERT_EQ(map.NumberOfShots(), 2) << map.NumberOfShots() << "==" << 2 ;
  const auto shot_id = "0";
  ASSERT_EQ(shot1, map.GetShot("0"));
  ASSERT_EQ(shot1->id_, shot_id);
  ASSERT_EQ(shot1->unique_id_, 0);
}

TEST(Map, TestShotGetter)
{
  auto map = CreateDefaultMap(1);
  const auto shot_id = "0";
  auto* shot1 = map.CreateShot(shot_id, "0");
  ASSERT_EQ(shot1, map.GetShot(shot_id));
  ASSERT_TRUE(map.HasShot(shot_id));
  ASSERT_ANY_THROW(map.GetShot("invalid_shot_id"));
}

TEST(Map, TestLandmarkGetter)
{
  auto map = map::Map();
  auto lm = map.CreateLandmark("0", Vec3d::Random());
  ASSERT_EQ(lm->id_, "0");
  ASSERT_EQ(lm, map.GetLandmark("0"));
  ASSERT_EQ(map.GetLandmark("0")->id_, "0");
  ASSERT_TRUE(map.HasLandmark("0"));
}

TEST(Map, TestCreatePoints)
{
  auto map = CreateDefaultMap(1);
  constexpr auto n_landmarks = 10;
  for (size_t pt_id = 0; pt_id < n_landmarks; ++pt_id)
  {
    const Vec3d coord = Vec3d::Random();
    const auto s_pt_id = std::to_string(pt_id);
    auto* pt = map.CreateLandmark(s_pt_id, coord);
    ASSERT_TRUE(map.HasLandmark(s_pt_id));
    ASSERT_EQ(s_pt_id, pt->id_);
    ASSERT_EQ(coord, pt->GetGlobalPos());
  }
  ASSERT_EQ(map.NumberOfLandmarks(), n_landmarks);
}


TEST(Map, TestInvalidIdAccess)
{
  auto map = map::Map();
  ASSERT_ANY_THROW(map.GetCamera("invalid"));
  ASSERT_ANY_THROW(map.GetLandmark("invalid"));
  ASSERT_ANY_THROW(map.GetShot("invalid"));
}


TEST(Map, TestDeletePoints)
{
  auto map = CreateDefaultMap(1, std::map<map::CameraId, size_t>(), 5);
  ASSERT_EQ(map.NumberOfLandmarks(), 5);
  map.RemoveLandmark("0");
  ASSERT_FALSE(map.HasLandmark("0"));
  ASSERT_EQ(map.NumberOfLandmarks(), 4);
  map.RemoveLandmark("0");
  ASSERT_EQ(map.NumberOfLandmarks(), 4);
  map.RemoveLandmark("1");
  ASSERT_FALSE(map.HasLandmark("1"));
  ASSERT_EQ(map.NumberOfLandmarks(), 3);
  map.ClearObservationsAndLandmarks();
  ASSERT_EQ(map.NumberOfLandmarks(), 0);

}



TEST(Map, SmallProblem)
{
  auto map = map::Map();
  constexpr auto n_points{300};
  constexpr auto n_shots{20};
  Camera camera = Camera::CreatePerspectiveCamera(0.5, 0, 0);
  camera.width = 640;
  camera.height = 480;
  camera.id = "cam1";
  auto* my_cam = map.CreateCamera(camera);
  for (auto i = 0; i < n_shots; ++i)
  {
    const auto shot_id = "shot"+std::to_string(i);
    map.CreateShot(shot_id, camera.id);
  }
  ASSERT_EQ(map.NumberOfShots(), n_shots);
  auto& shots = map.GetAllShots();
  ASSERT_EQ(shots.size(), n_shots);
  for (auto i = 0; i < n_points; ++i)
  {
    Eigen::Vector3d pos = Eigen::Vector3d::Random();
    const auto lm_id = std::to_string(i);
    auto* lm = map.CreateLandmark(std::to_string(i), pos);
    //all shots see all landmarks
    Observation obs(100,200, 0.5, 255,255,255,i);
    for (auto& shot_pair : shots)
    {
      map.AddObservation(&shot_pair.second,lm,obs);
    }
    ASSERT_EQ(lm->NumberOfObservations(), n_shots);
  }
  ASSERT_EQ(map.NumberOfLandmarks(), n_points);
  ASSERT_EQ(map.GetAllLandmarks().size(), n_points);
  for (const auto& shot_pair : shots)
  {
    ASSERT_EQ(shot_pair.second.ComputeNumValidLandmarks(), n_points);
  }
}
}  // namespace