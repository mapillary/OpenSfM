#include <foundation/stl_extensions.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <map/rig.h>

class RigCameraFixture : public ::testing::Test {
 public:
  RigCameraFixture() : rig_camera_id("rig_camera") {
    rig_camera_pose.SetOrigin(Vec3d::Random());
    rig_camera.pose = rig_camera_pose;
    rig_camera.id = rig_camera_id;
  }

  geometry::Camera camera;
  geometry::Pose rig_camera_pose;
  map::RigCameraId rig_camera_id;
  map::RigCamera rig_camera;
};

class SharedRigInstanceFixture : public RigCameraFixture {
 public:
  SharedRigInstanceFixture()
      : instance1(1),
        shot_instance1("1", camera, geometry::Pose()),
        instance2(2),
        shot_instance2("2", camera, geometry::Pose()) {
    instance_pose1.SetWorldToCamRotation(Vec3d::Random());
    instance_pose1.SetOrigin(Vec3d::Random());
    instance_pose2.SetWorldToCamRotation(Vec3d::Random());
    instance_pose2.SetOrigin(Vec3d::Random());
  }

  void ComparePoses(const geometry::Pose& expected,
                    const geometry::Pose& result) {
    const auto expected_o = expected.GetOrigin();
    const auto result_o = result.GetOrigin();
    for (int i = 0; i < 3; ++i) {
      ASSERT_NEAR(expected_o[i], result_o[i], 1e-10);
    }

    const auto expected_r = expected.RotationCameraToWorld();
    const auto result_r = result.RotationCameraToWorld();
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        ASSERT_NEAR(expected_r(i, j), result_r(i, j), 1e-10);
      }
    }
  }
  geometry::Pose instance_pose1;
  map::RigInstance instance1;
  map::Shot shot_instance1;

  geometry::Pose instance_pose2;
  map::RigInstance instance2;
  map::Shot shot_instance2;
};

TEST_F(SharedRigInstanceFixture, SetPose) {
  instance1.SetPose(instance_pose1);
  ComparePoses(instance_pose1, instance1.GetPose());
}

TEST_F(SharedRigInstanceFixture, AddShot) {
  instance1.AddShot(&rig_camera, &shot_instance1);
  ASSERT_EQ(1, instance1.GetShots().size());
  ASSERT_EQ(1, instance1.GetRigCameras().size());
}

TEST_F(SharedRigInstanceFixture, GetRigCameras) {
  instance1.AddShot(&rig_camera, &shot_instance1);
  auto added_rig_camera = instance1.GetRigCameras().at(shot_instance1.id_);
  ASSERT_EQ(rig_camera.id, added_rig_camera->id);
  ComparePoses(rig_camera.pose, added_rig_camera->pose);
}

class SharedRigInstanceWithShotsFixture : public SharedRigInstanceFixture {
 public:
  SharedRigInstanceWithShotsFixture() {
    instance1.SetPose(instance_pose1);
    instance2.SetPose(instance_pose2);
    instance1.AddShot(&rig_camera, &shot_instance1);
    instance2.AddShot(&rig_camera, &shot_instance2);
  }
};

TEST_F(SharedRigInstanceWithShotsFixture, MakesShotInRig) {
  ASSERT_TRUE(shot_instance1.IsInRig());
}

TEST_F(SharedRigInstanceWithShotsFixture, MakesShotReturnsRigCamera) {
  ASSERT_EQ(rig_camera_id, shot_instance1.GetRigCameraId());
}

TEST_F(SharedRigInstanceWithShotsFixture, MakesShotReturnsRigInstance) {
  ASSERT_EQ(1, shot_instance1.GetRigInstanceId());
  ASSERT_EQ(2, shot_instance2.GetRigInstanceId());
}

TEST_F(SharedRigInstanceWithShotsFixture, MakesShotThrowWhenSettingPose) {
  ASSERT_THROW(shot_instance1.SetPose(geometry::Pose()), std::runtime_error);
}

TEST_F(SharedRigInstanceWithShotsFixture, MakesShotThrowWhenGettingRefPose) {
  ASSERT_ANY_THROW(shot_instance1.GetPose()->SetOrigin(Vec3d::Random()));
}

TEST_F(SharedRigInstanceWithShotsFixture, MakesShotPoseDependsOnRig) {
  const auto shot_pose = foundation::as_const(shot_instance1).GetPose();
  const auto shot_pose_in_rig = rig_camera_pose.Compose(instance_pose1);
  ComparePoses(shot_pose_in_rig, *shot_pose);
}

TEST_F(SharedRigInstanceWithShotsFixture,
       UpdatesShotPoseWhenUpdatingRigCameraPose) {
  geometry::Pose new_pose;
  new_pose.SetOrigin(Vec3d::Random());
  instance1.UpdateRigCameraPose(rig_camera_id, new_pose);

  ComparePoses(new_pose.Compose(instance_pose1),
               *foundation::as_const(shot_instance1).GetPose());
  ComparePoses(new_pose.Compose(instance_pose2),
               *foundation::as_const(shot_instance2).GetPose());
}

TEST_F(SharedRigInstanceWithShotsFixture, UpdatesShotPoseWhenUpdatingShot) {
  geometry::Pose new_pose;
  new_pose.SetOrigin(Vec3d::Random());
  instance1.UpdateInstancePoseWithShot(shot_instance1.id_, new_pose);

  ComparePoses(new_pose, *foundation::as_const(shot_instance1).GetPose());
}

TEST_F(SharedRigInstanceWithShotsFixture,
       UpdatesShotPoseWhenUpdatingInstancePose) {
  geometry::Pose new_pose;
  new_pose.SetOrigin(Vec3d::Random());
  instance1.SetPose(new_pose);
  instance2.SetPose(new_pose);

  ComparePoses(*foundation::as_const(shot_instance2).GetPose(),
               *foundation::as_const(shot_instance1).GetPose());
}
