#include <ceres/jet.h>
#include <ceres/rotation.h>
#include <geometry/covariance.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <unsupported/Eigen/AutoDiff>

#include "geometry/transformations_functions.h"

class CovarianceFixture : public ::testing::Test {
 public:
  CovarianceFixture()
      : camera(geometry::Camera::CreatePerspectiveCamera(focal, 0.0, 0.0)),
        pose(geometry::Pose(Vec3d(0., 0., 0.), Vec3d(0, 0, 0))) {
    point_adiff[0].value() = point[0] = 0.;
    point_adiff[1].value() = point[1] = 0.;
    point_adiff[2].value() = point[2] = 3.;
    observation << 0.0, 0.0;
  }

  void RunAutodiffEval(const geometry::Camera& camera,
                       const geometry::Pose& pose) {
    // Prepare Eigen's Autodiff structures
    for (int i = 0; i < 3; ++i) {
      point_adiff[i].derivatives() = VecXd::Unit(3, i);
    }

    VecX<AScalar> pose_adiff(6);
    pose_adiff.segment<3>(3) = pose.TranslationCameraToWorld();
    pose_adiff.segment<3>(0) = pose.RotationCameraToWorldMin();
    for (int i = 0; i < 6; ++i) {
      pose_adiff(i).derivatives().resize(3);
      pose_adiff(i).derivatives().setZero();
    }

    VecX<AScalar> camera_adiff(3);
    const VecXd camera_params = camera.GetParametersValues();
    for (int i = 0; i < camera_params.size(); ++i) {
      camera_adiff(i).value() = camera_params(i);
      camera_adiff(i).derivatives().resize(3);
      camera_adiff(i).derivatives().setZero();
    }

    // Run project with Autodiff types to get expected jacobian
    AScalar transformed[3];
    geometry::PoseFunctor::Forward(point_adiff, pose_adiff.data(),
                                   &transformed[0]);
    geometry::Dispatch<geometry::ProjectFunction>(
        camera.GetProjectionType(), transformed, camera_adiff.data(),
        projection_expected);
  }

  template <class MAT>
  void CheckJacobian(const MAT& jacobian) {
    const double eps = 1e-12;
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 3; ++j) {
        ASSERT_NEAR(projection_expected[i].derivatives()(j), jacobian(i, j),
                    eps);
      }
      std::cout << projection_expected[i].value() << std::endl;
    }
  }

  const double focal{1.0};
  double point[3];
  typedef Eigen::AutoDiffScalar<VecXd> AScalar;
  AScalar point_adiff[3];
  AScalar projection_expected[2];
  Vec2d observation;

  const geometry::Camera camera;
  const geometry::Pose pose;
};

TEST_F(CovarianceFixture, EvaluatesPointJacobian) {
  Vec3d point_tmp(point[0], point[1], point[2]);

  const auto result = geometry::covariance::ComputeJacobianReprojectionError(
      camera, pose, observation, point_tmp);

  RunAutodiffEval(camera, pose);
  CheckJacobian(result.first);
  for (int i = 0; i < 2; ++i) {
    ASSERT_NEAR(0., result.second(i), 1e-10);
  }
}

TEST_F(CovarianceFixture, EvaluatesPointCovarianceKO) {
  Vec3d point_tmp(point[0], point[1], point[2]);

  auto covariance = geometry::covariance::ComputePointInverseCovariance(
                        {camera}, {pose}, {observation}, point_tmp)
                        .first;

  // Non-determined covariance for 1 projection
  ASSERT_TRUE(covariance.determinant() < 1e-12);
}

TEST_F(CovarianceFixture, EvaluatesPointCovarianceOK) {
  geometry::Pose pose_rotated_y;
  pose_rotated_y.SetWorldToCamRotation(Vec3d(0, -M_PI_2, 0));
  pose_rotated_y.SetOrigin(Vec3d(4, 0, 3));
  Vec3d point_tmp(point[0], point[1], point[2]);

  auto covariance = geometry::covariance::ComputePointInverseCovariance(
                        {camera, camera}, {pose, pose_rotated_y},
                        {observation, observation}, point_tmp)
                        .first;

  // Two pose looking with an angle of 90 degres
  ASSERT_TRUE(covariance.determinant() > 1e-12);
}

TEST_F(CovarianceFixture, EvaluatesPointCovarianceSmallBaseline) {
  const double baseline = 1e-5;
  const double angle = M_PI_2 - std::abs(std::atan2(point[2], baseline));

  geometry::Pose pose_rotated_y;
  pose_rotated_y.SetWorldToCamRotation(Vec3d(0, angle, 0));
  pose_rotated_y.SetOrigin(Vec3d(baseline, 0, 0));
  Vec3d point_tmp(point[0], point[1], point[2]);

  auto covariance = geometry::covariance::ComputePointInverseCovariance(
                        {camera, camera}, {pose, pose_rotated_y},
                        {observation, observation}, point_tmp)
                        .first;

  ASSERT_TRUE(covariance.determinant() < 1e-12);
}
