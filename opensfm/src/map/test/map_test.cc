#include <map/map.h>
#include <geometry/camera.h>
#include <sfm/observation.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>


namespace {
// TEST(Map, CreateAndDeleteShots) {}

TEST(Map, CreateCamera) {
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

TEST(Map, CreateAndDeleteShots) 
{
  Camera camera = Camera::CreatePerspectiveCamera(0.5, 0, 0);
  camera.width = 640;
  camera.height = 480;
  camera.id = "cam1";
  Camera camera2 = Camera::CreatePerspectiveCamera(0.5, 0, 0);
  camera2.width = 640;
  camera2.height = 480;
  camera2.id = "cam2";

  auto map = map::Map();
  constexpr auto n_shots{20};
  auto* my_cam = map.CreateCamera(camera);
  map.CreateCamera(camera2);
  for (auto i = 0; i < n_shots; ++i)
  {
    const auto shot_id = "shot"+std::to_string(i);
    auto* shot = map.CreateShot(shot_id, i < n_shots/2 ? camera.id : camera2.id);
    ASSERT_EQ(shot, map.GetShot(shot_id));
    ASSERT_EQ(shot->id_, shot_id);
    ASSERT_EQ(shot->unique_id_, i);
    ASSERT_TRUE(map.HasShot(shot_id));
  }
  ASSERT_EQ(map.NumberOfShots(), n_shots);
  ASSERT_EQ(map.GetAllShots().size(), n_shots);
  for (auto i = 0; i < n_shots; ++i)
  {
    const auto shot_id = "shot"+std::to_string(i);
    map.RemoveShot(shot_id);
  }
  ASSERT_EQ(map.NumberOfShots(), 0);

  // Add another set of shots
  for (auto i = 0; i < n_shots; ++i)
  {
    const auto shot_id = "shot"+std::to_string(i);
    auto* shot = map.CreateShot(shot_id, my_cam);
    ASSERT_EQ(shot, map.GetShot(shot_id));
    //Creating the same shot again, should yield the same pointer
    ASSERT_EQ(shot, map.CreateShot(shot_id, my_cam));
    ASSERT_EQ(shot->id_, shot_id);
    ASSERT_EQ(shot->unique_id_, n_shots+i);
    ASSERT_EQ(shot->GetCamera()->id, my_cam->id);
  }
  ASSERT_EQ(map.NumberOfShots(), n_shots);
  ASSERT_EQ(map.GetShot("invalid_shot_id"), nullptr);
}

TEST(Map, CreateAndDeletePoints)
{
  auto map = map::Map();
  constexpr auto n_points{300};
  for (auto i = 0; i < n_points; ++i)
  {
    Eigen::Vector3d pos = Eigen::Vector3d::Random();
    const auto lm_id = std::to_string(i);
    auto* lm = map.CreateLandmark(std::to_string(i), pos);
    ASSERT_EQ(pos, lm->GetGlobalPos());
    ASSERT_EQ(lm, map.GetLandmark(lm_id));
    ASSERT_EQ(lm, map.CreateLandmark(std::to_string(i), pos));
    ASSERT_EQ(lm->unique_id_, i);
    ASSERT_EQ(map.NumberOfLandmarks(), i+1);
    ASSERT_TRUE(map.HasLandmark(lm_id));
  }
  ASSERT_EQ(map.NumberOfLandmarks(), n_points);
  ASSERT_EQ(map.GetAllLandmarks().size(), n_points);
  ASSERT_EQ(map.GetLandmark("invalid_lm"), nullptr);
  //Delete the first 150
  for (auto i = 0; i < int(n_points/2); ++i)
  {
    map.RemoveLandmark(std::to_string(i));
  }
  ASSERT_EQ(map.NumberOfLandmarks(), int(n_points/2));
  for (auto i=int(n_points/2); i < n_points; ++i)
  {
    const auto lm_id = std::to_string(i);
    auto* lm = map.GetLandmark(lm_id);
    map.RemoveLandmark(lm);
    ASSERT_FALSE(map.HasLandmark(lm_id));
  }
  ASSERT_EQ(map.NumberOfLandmarks(),0);
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
  const auto& shots = map.GetAllShots();
  for (auto i = 0; i < n_points; ++i)
  {
    Eigen::Vector3d pos = Eigen::Vector3d::Random();
    const auto lm_id = std::to_string(i);
    auto* lm = map.CreateLandmark(std::to_string(i), pos);
    //all shots see all landmarks
    Observation obs(100,200, 0.5, 255,255,255,i);
    for (const auto& shot_pair : shots)
    {
      map.AddObservation(shot_pair.second.get(),lm,obs);
    }
    ASSERT_EQ(lm->NumberOfObservations(), n_shots);
  }
  for (const auto& shot_pair : shots)
  {
    ASSERT_EQ(shot_pair.second->ComputeNumValidLandmarks(), n_points);
  }
}
}  // namespace