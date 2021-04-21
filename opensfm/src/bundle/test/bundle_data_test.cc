#include <bundle/bundle_adjuster.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>

namespace geometry {
bool operator==(const geometry::Pose& p1, const geometry::Pose& p2) {
  const double eps = 1e-15;
  const auto identity = p1.Compose(p2.Inverse());
  return identity.GetOrigin().norm() < eps &&
         identity.RotationCameraToWorldMin().norm() < eps;
}
}  // namespace geometry

TEST(DataNode, ReturnCorrectID) {
  bundle::DataNode node("node_id");
  ASSERT_EQ("node_id", node.GetID());
}

class BAPoseWithPriorFixtureBase {
 public:
  BAPoseWithPriorFixtureBase()
      : id("pose_id"),
        pose(Vec3d(0.1, 0.2, 0.3), Vec3d(0.1, 0.2, 0.3)),
        pose_sigma(Vec3d(0.1, 0.1, 0.1), Vec3d(0.1, 0.1, 0.1)),
        ba_pose(id, pose, pose, pose_sigma) {}

  std::string id;
  geometry::Pose pose;
  geometry::Pose pose_sigma;
  bundle::Pose ba_pose;
};

class BAPoseWithPriorFixture : public ::testing::Test,
                               public BAPoseWithPriorFixtureBase {};

TEST_F(BAPoseWithPriorFixture, ReturnValueData) {
  VecNd<6> expected;
  expected.segment(0, 3) = pose.RotationCameraToWorldMin();
  expected.segment(3, 3) = pose.GetOrigin();
  ASSERT_TRUE(expected.isApprox(ba_pose.GetValueData()));
}

TEST_F(BAPoseWithPriorFixture, ReturnValue) {
  ASSERT_EQ(pose, ba_pose.GetValue());
}

TEST_F(BAPoseWithPriorFixture, HasPrior) { ASSERT_TRUE(ba_pose.HasPrior()); }

TEST_F(BAPoseWithPriorFixture, ReturnPriorData) {
  VecNd<6> expected;
  expected.segment(0, 3) = pose.RotationCameraToWorldMin();
  expected.segment(3, 3) = pose.GetOrigin();
  ASSERT_TRUE(expected.isApprox(ba_pose.GetPriorData()));
}

TEST_F(BAPoseWithPriorFixture, SetPriorData) {
  geometry::Pose new_value(Vec3d(0.5, 0.6, 0.7), Vec3d(7, 8, 9));
  ba_pose.SetPrior(new_value);

  VecNd<6> expected;
  expected.segment(0, 3) = new_value.RotationCameraToWorldMin();
  expected.segment(3, 3) = new_value.GetOrigin();
  ASSERT_TRUE(expected.isApprox(ba_pose.GetPriorData()));
}

TEST_F(BAPoseWithPriorFixture, ReturnSigmaData) {
  VecNd<6> expected;
  expected.segment(0, 3) = pose_sigma.RotationCameraToWorldMin();
  expected.segment(3, 3) = pose_sigma.GetOrigin();
  ASSERT_TRUE(expected.isApprox(ba_pose.GetSigmaData()));
}

TEST_F(BAPoseWithPriorFixture, SetSigmaData) {
  geometry::Pose new_value(Vec3d(0.5, 0.6, 0.7), Vec3d(7, 8, 9));
  ba_pose.SetSigma(new_value);

  VecNd<6> expected;
  expected.segment(0, 3) = new_value.RotationCameraToWorldMin();
  expected.segment(3, 3) = new_value.GetOrigin();
  ASSERT_TRUE(expected.isApprox(ba_pose.GetSigmaData()));
}

TEST_F(BAPoseWithPriorFixture, SetCovariance) {
  MatXd covariance(6, 6);
  covariance.setIdentity();
  ba_pose.SetCovariance(covariance);
  ASSERT_TRUE(ba_pose.HasCovariance());
}

TEST_F(BAPoseWithPriorFixture, HasCovariance) {
  ASSERT_FALSE(ba_pose.HasCovariance());
}

TEST_F(BAPoseWithPriorFixture, ReturnsCovariance) {
  MatXd covariance(6, 6);
  covariance.setIdentity();
  ba_pose.SetCovariance(covariance);
  ASSERT_TRUE(covariance.isApprox(ba_pose.GetCovariance()));
}

TEST_F(BAPoseWithPriorFixture, ReturnsParametersToOptimize) {
  std::vector<int> parameters = {0, 1, 2, 3, 4, 5};
  ASSERT_EQ(parameters, ba_pose.GetParametersToOptimize());
}

TEST_F(BAPoseWithPriorFixture, SetParametersToOptimize) {
  std::vector<int> parameters = {0, 1, 2};
  ba_pose.SetParametersToOptimize(parameters);
  ASSERT_EQ(parameters, ba_pose.GetParametersToOptimize());
}

class BACameraWithPriorFixtureBase {
 public:
  BACameraWithPriorFixtureBase()
      : id("camera_id"),
        camera(geometry::Camera::CreatePerspectiveCamera(0.1, 0.2, 0.3)),
        camera_sigma(
            geometry::Camera::CreatePerspectiveCamera(0.01, 0.01, 0.01)),
        ba_camera(id, camera, camera, camera_sigma) {}
  std::string id;
  geometry::Camera camera;
  geometry::Camera camera_sigma;
  bundle::Camera ba_camera;
};

class BACameraWithPriorFixture : public BACameraWithPriorFixtureBase,
                                 public ::testing::Test {};

TEST_F(BACameraWithPriorFixture, ReturnValueData) {
  VecNd<3> expected;
  expected << 0.2, 0.3, 0.1;
  ASSERT_TRUE(expected.isApprox(ba_camera.GetValueData()));
}

TEST_F(BACameraWithPriorFixture, ReturnPriorData) {
  VecNd<3> expected;
  expected << 0.2, 0.3, 0.1;
  ASSERT_TRUE(expected.isApprox(ba_camera.GetPriorData()));
}

TEST_F(BACameraWithPriorFixture, ReturnSigmaData) {
  VecNd<3> expected;
  expected << 0.01, 0.01, 0.01;
  ASSERT_TRUE(expected.isApprox(ba_camera.GetSigmaData()));
}

class BAShotFixture : public BACameraWithPriorFixtureBase,
                      public BAPoseWithPriorFixtureBase,
                      public ::testing::Test {
 public:
  BAShotFixture() : id("shot_id"), shot(id, &ba_camera, pose) {}
  std::string id;
  bundle::Shot shot;
};

TEST_F(BAShotFixture, ReturnsPose) {
  ASSERT_EQ(pose, shot.GetPose()->GetValue());
}

TEST_F(BAShotFixture, ReturnsCamera) {
  ASSERT_EQ(&ba_camera, shot.GetCamera());
}

class BARigShotFixture : public BACameraWithPriorFixtureBase,
                         public BAPoseWithPriorFixtureBase,
                         public ::testing::Test {
 public:
  BARigShotFixture()
      : id("rig_shot_id"), rig_shot(id, &ba_camera, &ba_pose, &ba_pose) {}
  std::string id;
  bundle::RigShot rig_shot;
};

TEST_F(BARigShotFixture, ReturnsCamera) {
  ASSERT_EQ(&ba_camera, rig_shot.GetCamera());
}

TEST_F(BARigShotFixture, ReturnsRigCamera) {
  ASSERT_EQ(&ba_pose, rig_shot.GetRigCamera());
}

TEST_F(BARigShotFixture, ReturnsRigInstance) {
  ASSERT_EQ(&ba_pose, rig_shot.GetRigInstance());
}
