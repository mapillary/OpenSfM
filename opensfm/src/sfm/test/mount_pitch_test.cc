#include <geometry/camera.h>
#include <geometry/pose.h>
#include <gtest/gtest.h>
#include <map/map.h>
#include <sfm/mount_pitch.h>

#include <cmath>
#include <string>

namespace {

constexpr double kDegToRad = M_PI / 180.0;

// World-to-camera rotation of a camera looking along +X (or -X when
// `backward`), pitched nose-up by `pitch`, with no roll.
Mat3d PitchedCameraRotation(double pitch, bool backward = false) {
  const double c = std::cos(pitch);
  const double s = std::sin(pitch);
  const double sign = backward ? -1.0 : 1.0;
  const Vec3d forward(sign * c, 0, s);
  const Vec3d down(sign * s, 0, -c);
  Mat3d rotation;
  rotation.row(0) = down.cross(forward);
  rotation.row(1) = down;
  rotation.row(2) = forward;
  return rotation;
}

class MountPitchTest : public ::testing::Test {
 protected:
  MountPitchTest() {
    auto camera = geometry::Camera::CreatePerspectiveCamera(0.5, 0, 0);
    camera.id = "camera";
    map.CreateCamera(camera);
    map::RigCamera rig_camera;
    rig_camera.id = "rig_camera";
    map.CreateRigCamera(rig_camera);
  }

  // Adds `count` shots driving along +X, one meter apart. `world` maps the
  // level world frame to the reconstruction frame.
  void AddSequence(const std::string& sequence, int count, double pitch,
                   const Mat3d& world = Mat3d::Identity(),
                   bool backward = false) {
    for (int i = 0; i < count; ++i) {
      const auto id = sequence + "_" + std::to_string(i);
      map.CreateRigInstance(id);
      const Mat3d rotation =
          PitchedCameraRotation(pitch, backward) * world.transpose();
      geometry::Pose pose(rotation);
      pose.SetOrigin(world * Vec3d(i, 0, 0));
      auto& shot = map.CreateShot(id, "camera", "rig_camera", id, pose);
      shot.GetShotMeasurements().capture_time_.SetValue(i);
      shot.GetShotMeasurements().sequence_key_.SetValue(sequence);
    }
  }

  map::Map map;
  sfm::mount_pitch::Options options;
};

TEST_F(MountPitchTest, RecoversNoseUpPitch) {
  AddSequence("a", 20, 10 * kDegToRad);

  const auto pitches = sfm::mount_pitch::EstimateMountPitches(map, options);

  ASSERT_EQ(20, pitches.size());
  EXPECT_NEAR(10 * kDegToRad, pitches.at("a_0"), 1e-9);
}

TEST_F(MountPitchTest, IgnoresReconstructionFrame) {
  const Mat3d world =
      (Eigen::AngleAxisd(0.7, Vec3d(1, 2, 3).normalized())).toRotationMatrix();
  AddSequence("a", 20, -5 * kDegToRad, world);

  const auto pitches = sfm::mount_pitch::EstimateMountPitches(map, options);

  EXPECT_NEAR(-5 * kDegToRad, pitches.at("a_7"), 1e-9);
}

TEST_F(MountPitchTest, HandlesRearFacingCamera) {
  AddSequence("a", 20, 8 * kDegToRad, Mat3d::Identity(), /*backward=*/true);

  const auto pitches = sfm::mount_pitch::EstimateMountPitches(map, options);

  EXPECT_NEAR(8 * kDegToRad, pitches.at("a_3"), 1e-9);
}

TEST_F(MountPitchTest, EstimatesEachSequenceSeparately) {
  AddSequence("a", 20, 10 * kDegToRad);
  AddSequence("b", 20, 2 * kDegToRad);

  const auto pitches = sfm::mount_pitch::EstimateMountPitches(map, options);

  EXPECT_NEAR(10 * kDegToRad, pitches.at("a_5"), 1e-9);
  EXPECT_NEAR(2 * kDegToRad, pitches.at("b_5"), 1e-9);
}

TEST_F(MountPitchTest, SkipsShotsWithoutSequenceKey) {
  AddSequence("a", 20, 10 * kDegToRad);
  for (int i = 0; i < 20; ++i) {
    map.GetShot("a_" + std::to_string(i))
        .GetShotMeasurements()
        .sequence_key_.Reset();
  }

  EXPECT_TRUE(sfm::mount_pitch::EstimateMountPitches(map, options).empty());
}

TEST_F(MountPitchTest, SkipsShortSequences) {
  AddSequence("a", 5, 10 * kDegToRad);

  EXPECT_TRUE(sfm::mount_pitch::EstimateMountPitches(map, options).empty());
}

TEST_F(MountPitchTest, SkipsImplausiblePitch) {
  AddSequence("a", 20, 40 * kDegToRad);

  EXPECT_TRUE(sfm::mount_pitch::EstimateMountPitches(map, options).empty());
}

TEST_F(MountPitchTest, SkipsInconsistentPitch) {
  AddSequence("a", 20, 0);
  for (int i = 0; i < 20; ++i) {
    auto& shot = map.GetShot("a_" + std::to_string(i));
    const double pitch = (i % 2 == 0 ? 10 : -10) * kDegToRad;
    const Mat3d rotation = PitchedCameraRotation(pitch);
    geometry::Pose pose(rotation);
    pose.SetOrigin(Vec3d(i, 0, 0));
    shot.SetPose(pose);
  }

  EXPECT_TRUE(sfm::mount_pitch::EstimateMountPitches(map, options).empty());
}

}  // namespace
