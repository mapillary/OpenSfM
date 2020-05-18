#include <random>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <geometry/camera.h>

class CameraFixture : public ::testing::Test {
 public:
  CameraFixture() {
    pixels.resize(pixels_count, 2);

    std::mt19937 gen(42);
    std::uniform_real_distribution<> rand(-0.5, 0.5);
    for (int i = 0; i < pixels_count; ++i) {
      const auto x = rand(gen) * max_width;
      const auto y = rand(gen) * max_height;
      pixels.row(i) << x, y;
    }

    distortion.resize(5);
    distortion << -0.1, 0.03, 0.001, 0.001, 0.002;
    principal_point << 0.1, -0.05;

    new_distortion.resize(5);
    new_distortion << 1, 1, 1, 1, 1;
    new_principal_point << 0.02, -0.01;
  }

  double ComputeError(const Eigen::MatrixX2d& other) {
    double error = 0;
    for (int i = 0; i < pixels_count; ++i) {
      error += (other.row(i) - pixels.row(i)).norm();
    }
    return error/pixels_count;
  }

  int pixels_count{10000};
  Eigen::MatrixX2d pixels;

  double max_width{0.90};
  double max_height{0.75};
  double focal{0.4};
  double new_focal{0.8};
  double new_ar{0.9};

  int pixel_width{4000};
  int pixel_height{3000};

  Eigen::VectorXd distortion;
  Eigen::VectorXd new_distortion;
  Eigen::Vector2d principal_point;
  Eigen::Vector2d new_principal_point;
};

TEST_F(CameraFixture, PerspectiveIsConsistent){
  Camera camera = Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, BrownIsConsistent){
  Camera camera = Camera::CreateBrownCamera(focal, 1.0, principal_point, distortion);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, FisheyeIsConsistent){
  Camera camera = Camera::CreateFisheyeCamera(focal, distortion[0], distortion[1]);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, DualIsConsistent){
  Camera camera = Camera::CreateDualCamera(0.5, focal, distortion[0], distortion[1]);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, SphericalIsConsistent){
  Camera camera = Camera::CreateSphericalCamera();
  const auto projected = camera.Project(camera.Bearing(Eigen::Vector2d(0.2, 0.2)));
  ASSERT_EQ(projected(0), 0.2);
  ASSERT_EQ(projected(1), 0.2);
}

TEST_F(CameraFixture, PerspectiveCanSetDistortion){
  Camera camera = Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);
  camera.SetDistortion(new_distortion);

  Eigen::VectorXd expected_distortion(5);
  expected_distortion << 1, 1, 0, 0, 0;
  ASSERT_EQ(expected_distortion, camera.GetDistortion());
}

TEST_F(CameraFixture, FisheyeCanSetDistortion){
  Camera camera = Camera::CreateFisheyeCamera(focal, distortion[0], distortion[1]);
  camera.SetDistortion(new_distortion);

  Eigen::VectorXd expected_distortion(5);
  expected_distortion << 1, 1, 0, 0, 0;
  ASSERT_EQ(expected_distortion, camera.GetDistortion());
}

TEST_F(CameraFixture, DualCanSetDistortion){
  Camera camera = Camera::CreateDualCamera(0.5, focal, distortion[0], distortion[1]);
  camera.SetDistortion(new_distortion);

  Eigen::VectorXd expected_distortion(5);
  expected_distortion << 1, 1, 0, 0, 0;
  ASSERT_EQ(expected_distortion, camera.GetDistortion());
}

TEST_F(CameraFixture, BrownCanSetDistortion){
  Camera camera = Camera::CreateBrownCamera(focal, 1.0, principal_point, distortion);
  camera.SetDistortion(new_distortion);

  ASSERT_EQ(new_distortion, camera.GetDistortion());
}

TEST_F(CameraFixture, SphericalCanSetDistortion){
  Camera camera = Camera::CreateSphericalCamera();
  camera.SetDistortion(new_distortion);

  Eigen::VectorXd expected_distortion(5);
  expected_distortion << 0, 0, 0, 0, 0;
  ASSERT_EQ(expected_distortion, camera.GetDistortion());
}

TEST_F(CameraFixture, PerspectiveCanSetPrincipalPoint){
  Camera camera = Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);
  camera.SetPrincipalPoint(new_principal_point);

  const auto expected_principal_point = Eigen::Vector2d::Zero();
  ASSERT_EQ(expected_principal_point, camera.GetPrincipalPoint());
}

TEST_F(CameraFixture, FisheyeCanSetPrincipalPoint){
  Camera camera = Camera::CreateFisheyeCamera(focal, distortion[0], distortion[1]);
  camera.SetPrincipalPoint(new_principal_point);

  const auto expected_principal_point = Eigen::Vector2d::Zero();
  ASSERT_EQ(expected_principal_point, camera.GetPrincipalPoint());
}

TEST_F(CameraFixture, DualCanSetPrincipalPoint){
  Camera camera = Camera::CreateDualCamera(0.5, focal, distortion[0], distortion[1]);
  camera.SetPrincipalPoint(new_principal_point);

  const auto expected_principal_point = Eigen::Vector2d::Zero();
  ASSERT_EQ(expected_principal_point, camera.GetPrincipalPoint());
}

TEST_F(CameraFixture, BrownCanSetPrincipalPoint){
  Camera camera = Camera::CreateBrownCamera(focal, 1.0, principal_point, distortion);
  camera.SetPrincipalPoint(new_principal_point);

  ASSERT_EQ(new_principal_point, camera.GetPrincipalPoint());
}

TEST_F(CameraFixture, SphericalCanSetPrincipalPoint){
  Camera camera = Camera::CreateSphericalCamera();
  camera.SetPrincipalPoint(new_principal_point);

  const auto expected_principal_point = Eigen::Vector2d::Zero();
  ASSERT_EQ(expected_principal_point, camera.GetPrincipalPoint());
}

TEST_F(CameraFixture, PerspectiveCanSetFocalAndAspectRatio){
  Camera camera = Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);
  camera.SetFocal(new_focal);
  camera.SetAspectRatio(new_ar);

  ASSERT_EQ(new_focal, camera.GetFocal());
  ASSERT_EQ(1.0, camera.GetAspectRatio());
}

TEST_F(CameraFixture, FisheyeCanSetFocalAndAspectRatio){
  Camera camera = Camera::CreateFisheyeCamera(focal, distortion[0], distortion[1]);
  camera.SetFocal(new_focal);
  camera.SetAspectRatio(new_ar);

  ASSERT_EQ(new_focal, camera.GetFocal());
  ASSERT_EQ(1.0, camera.GetAspectRatio());
}

TEST_F(CameraFixture, DualCanSetFocalAndAspectRatio){
  Camera camera = Camera::CreateDualCamera(0.5, focal, distortion[0], distortion[1]);
  camera.SetFocal(new_focal);
  camera.SetAspectRatio(new_ar);

  ASSERT_EQ(new_focal, camera.GetFocal());
  ASSERT_EQ(1.0, camera.GetAspectRatio());
}

TEST_F(CameraFixture, BrownCanSetFocalAndAspectRatio){
  Camera camera = Camera::CreateBrownCamera(focal, 1.0, principal_point, distortion);
  camera.SetFocal(new_focal);
  camera.SetAspectRatio(new_ar);

  ASSERT_EQ(new_focal, camera.GetFocal());
  ASSERT_EQ(new_ar, camera.GetAspectRatio());
}

TEST_F(CameraFixture, SphericalCanSetFocalAndAspectRatio){
  Camera camera = Camera::CreateSphericalCamera();
  camera.SetFocal(new_focal);
  camera.SetAspectRatio(new_ar);

  ASSERT_EQ(1.0, camera.GetFocal());
  ASSERT_EQ(1.0, camera.GetAspectRatio());
}

TEST_F(CameraFixture, PerspectiveReturnCorrectK){
  Camera camera = Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);

  Eigen::Matrix3d expected = Eigen::Matrix3d::Identity();
  expected(0, 0) = expected(1, 1) = focal;
  ASSERT_EQ(expected, camera.GetProjectionMatrix());
}

TEST_F(CameraFixture, PerspectiveReturnCorrectKScaled){
  Camera camera = Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);

  Eigen::Matrix3d expected = Eigen::Matrix3d::Identity();
  expected(0, 0) = expected(1, 1) = focal*std::max(pixel_width, pixel_height);
  expected(0, 2) = pixel_width*0.5;
  expected(1, 2) = pixel_height*0.5;
  ASSERT_EQ(expected, camera.GetProjectionMatrixScaled(pixel_width, pixel_height));
}

TEST_F(CameraFixture, BrownReturnCorrectK){
  Camera camera = Camera::CreateBrownCamera(focal, new_ar, principal_point, distortion);

  Eigen::Matrix3d expected = Eigen::Matrix3d::Identity();
  expected(0, 0) = focal;
  expected(1, 1) = new_ar*focal;
  expected(0, 2) = principal_point(0);
  expected(1, 2) = principal_point(1);
  ASSERT_EQ(expected, camera.GetProjectionMatrix());
}

TEST_F(CameraFixture, BrownReturnCorrectKScaled){
  Camera camera = Camera::CreateBrownCamera(focal, new_ar, principal_point, distortion);

  Eigen::Matrix3d expected = Eigen::Matrix3d::Identity();
  const auto normalizer = std::max(pixel_width, pixel_height);
  expected(0, 0) = focal*normalizer;
  expected(1, 1) = (focal*new_ar)*normalizer;
  expected(0, 2) = principal_point(0)*normalizer + pixel_width*0.5;
  expected(1, 2) = principal_point(1)*normalizer + pixel_height*0.5;

  ASSERT_EQ(expected, camera.GetProjectionMatrixScaled(pixel_width, pixel_height));
}