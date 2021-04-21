#include <geometry/camera.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <random>

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

    distortion.resize(2);
    distortion << -0.1, 0.03;
    distortion_brown.resize(5);
    distortion_brown << -0.1, 0.03, 0.001, 0.001, 0.002;
    distortion_fisheye.resize(4);
    distortion_fisheye << -0.1, 0.03, 0.001, 0.005;
    distortion_fisheye62.resize(8);
    distortion_fisheye62 << -0.1, 0.03, 0.001, 0.005, 0.02, 0.001, 0.0007,
        -0.01;
    distortion_radial << 0.1, 0.03;
    principal_point << 0.1, -0.05;
  }

  double ComputeError(const MatX2d& other) {
    double error = 0;
    for (int i = 0; i < pixels_count; ++i) {
      error += (other.row(i) - pixels.row(i)).norm();
    }
    return error / pixels_count;
  }

  const int pixels_count{10000};
  const int pixel_width{4000};
  const int pixel_height{3000};

  const double max_width{0.90};
  const double max_height{0.75};
  const double focal{0.4};
  const double new_focal{0.8};
  const double new_ar{0.9};

  MatX2d pixels;
  VecXd distortion;
  VecXd distortion_brown;
  VecXd distortion_fisheye;
  VecXd distortion_fisheye62;
  Vec2d distortion_radial;
  Vec2d principal_point;

  double point[3];
  double projected[2];
};

TEST_F(CameraFixture, PerspectiveIsConsistent) {
  geometry::Camera camera = geometry::Camera::CreatePerspectiveCamera(
      focal, distortion[0], distortion[1]);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, BrownIsConsistent) {
  geometry::Camera camera = geometry::Camera::CreateBrownCamera(
      focal, 1.0, principal_point, distortion_brown);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, FisheyeIsConsistent) {
  geometry::Camera camera = geometry::Camera::CreateFisheyeCamera(
      focal, distortion[0], distortion[1]);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, FisheyeIsConsistentLargeFov) {
  double short_focal = 0.3;
  geometry::Camera camera =
      geometry::Camera::CreateFisheyeCamera(short_focal, 0.001, 0.001);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, FisheyeOpencvIsConsistent) {
  geometry::Camera camera = geometry::Camera::CreateFisheyeOpencvCamera(
      focal, 1.0, principal_point, distortion_fisheye);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, Fisheye62IsConsistent) {
  geometry::Camera camera = geometry::Camera::CreateFisheye62Camera(
      focal, 1.0, principal_point, distortion_fisheye62);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, DualIsConsistent) {
  geometry::Camera camera = geometry::Camera::CreateDualCamera(
      0.5, focal, distortion[0], distortion[1]);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, SphericalIsConsistent) {
  geometry::Camera camera = geometry::Camera::CreateSphericalCamera();
  const auto projected = camera.Project(camera.Bearing(Vec2d(0.2, 0.2)));
  ASSERT_EQ(projected(0), 0.2);
  ASSERT_EQ(projected(1), 0.2);
}

TEST_F(CameraFixture, RadialIsConsistent) {
  geometry::Camera camera = geometry::Camera::CreateRadialCamera(
      focal, 1.0, principal_point, distortion_radial);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, SimpleRadialIsConsistent) {
  geometry::Camera camera = geometry::Camera::CreateSimpleRadialCamera(
      focal, 1.0, principal_point, distortion_radial[0]);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, SphericalReturnCorrectTypes) {
  geometry::Camera camera = geometry::Camera::CreateSphericalCamera();
  const auto types = camera.GetParametersTypes();
  ASSERT_EQ(1, types.size());
  ASSERT_EQ(geometry::Camera::Parameters::None, types[0]);
}

TEST_F(CameraFixture, PerspectiveReturnCorrectTypes) {
  geometry::Camera camera = geometry::Camera::CreatePerspectiveCamera(
      focal, distortion[0], distortion[1]);
  const auto types = camera.GetParametersTypes();
  const auto expected = std::vector<geometry::Camera::Parameters>(
      {geometry::Camera::Parameters::K1, geometry::Camera::Parameters::K2,
       geometry::Camera::Parameters::Focal});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, FisheyeReturnCorrectTypes) {
  geometry::Camera camera = geometry::Camera::CreateFisheyeCamera(
      focal, distortion[0], distortion[1]);
  const auto types = camera.GetParametersTypes();
  const auto expected = std::vector<geometry::Camera::Parameters>(
      {geometry::Camera::Parameters::K1, geometry::Camera::Parameters::K2,
       geometry::Camera::Parameters::Focal});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, FisheyeOpencvReturnCorrectTypes) {
  geometry::Camera camera = geometry::Camera::CreateFisheyeOpencvCamera(
      focal, 1.0, principal_point, distortion_fisheye);
  const auto types = camera.GetParametersTypes();
  const auto expected = std::vector<geometry::Camera::Parameters>(
      {geometry::Camera::Parameters::K1, geometry::Camera::Parameters::K2,
       geometry::Camera::Parameters::K3, geometry::Camera::Parameters::K4,
       geometry::Camera::Parameters::Focal,
       geometry::Camera::Parameters::AspectRatio,
       geometry::Camera::Parameters::Cx, geometry::Camera::Parameters::Cy});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, Fisheye62ReturnCorrectTypes) {
  geometry::Camera camera = geometry::Camera::CreateFisheye62Camera(
      focal, 1.0, principal_point, distortion_fisheye62);
  const auto types = camera.GetParametersTypes();
  const auto expected = std::vector<geometry::Camera::Parameters>(
      {geometry::Camera::Parameters::K1, geometry::Camera::Parameters::K2,
       geometry::Camera::Parameters::K3, geometry::Camera::Parameters::K4,
       geometry::Camera::Parameters::K5, geometry::Camera::Parameters::K6,
       geometry::Camera::Parameters::P1, geometry::Camera::Parameters::P2,
       geometry::Camera::Parameters::Focal,
       geometry::Camera::Parameters::AspectRatio,
       geometry::Camera::Parameters::Cx, geometry::Camera::Parameters::Cy});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, RadialReturnCorrectTypes) {
  geometry::Camera camera = geometry::Camera::CreateRadialCamera(
      focal, 1.0, principal_point, distortion_radial);
  const auto types = camera.GetParametersTypes();
  const auto expected = std::vector<geometry::Camera::Parameters>(
      {geometry::Camera::Parameters::K1, geometry::Camera::Parameters::K2,
       geometry::Camera::Parameters::Focal,
       geometry::Camera::Parameters::AspectRatio,
       geometry::Camera::Parameters::Cx, geometry::Camera::Parameters::Cy});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, SimpleRadialReturnCorrectTypes) {
  geometry::Camera camera = geometry::Camera::CreateSimpleRadialCamera(
      focal, 1.0, principal_point, distortion_radial[0]);
  const auto types = camera.GetParametersTypes();
  const auto expected = std::vector<geometry::Camera::Parameters>(
      {geometry::Camera::Parameters::K1, geometry::Camera::Parameters::Focal,
       geometry::Camera::Parameters::AspectRatio,
       geometry::Camera::Parameters::Cx, geometry::Camera::Parameters::Cy});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, BrownReturnCorrectTypes) {
  geometry::Camera camera = geometry::Camera::CreateBrownCamera(
      focal, 1.0, principal_point, distortion_brown);
  const auto types = camera.GetParametersTypes();

  const auto expected = std::vector<geometry::Camera::Parameters>(
      {geometry::Camera::Parameters::K1, geometry::Camera::Parameters::K2,
       geometry::Camera::Parameters::K3, geometry::Camera::Parameters::P1,
       geometry::Camera::Parameters::P2, geometry::Camera::Parameters::Focal,
       geometry::Camera::Parameters::AspectRatio,
       geometry::Camera::Parameters::Cx, geometry::Camera::Parameters::Cy});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, DualReturnCorrectTypes) {
  geometry::Camera camera = geometry::Camera::CreateDualCamera(
      0.5, focal, distortion[0], distortion[1]);
  const auto types = camera.GetParametersTypes();
  const auto expected = std::vector<geometry::Camera::Parameters>(
      {geometry::Camera::Parameters::Transition,
       geometry::Camera::Parameters::K1, geometry::Camera::Parameters::K2,
       geometry::Camera::Parameters::Focal});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, PerspectiveReturnCorrectValues) {
  geometry::Camera camera = geometry::Camera::CreatePerspectiveCamera(
      focal, distortion[0], distortion[1]);
  const auto values = camera.GetParametersValues();

  VecXd expected(3);
  expected << distortion[0], distortion[1], focal;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, FisheyeReturnCorrectValues) {
  geometry::Camera camera = geometry::Camera::CreateFisheyeCamera(
      focal, distortion[0], distortion[1]);
  const auto values = camera.GetParametersValues();

  VecXd expected(3);
  expected << distortion[0], distortion[1], focal;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, FisheyeOpencvReturnCorrectValues) {
  geometry::Camera camera = geometry::Camera::CreateFisheyeOpencvCamera(
      focal, 1.0, principal_point, distortion_fisheye);
  const auto values = camera.GetParametersValues();

  VecXd expected(8);
  expected << distortion_fisheye, focal, 1.0, principal_point;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, Fisheye62ReturnCorrectValues) {
  geometry::Camera camera = geometry::Camera::CreateFisheye62Camera(
      focal, 1.0, principal_point, distortion_fisheye62);
  const auto values = camera.GetParametersValues();

  VecXd expected(12);
  expected << distortion_fisheye62, focal, 1.0, principal_point;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, RadialReturnCorrectValues) {
  geometry::Camera camera = geometry::Camera::CreateRadialCamera(
      focal, 1.0, principal_point, distortion_radial);
  const auto values = camera.GetParametersValues();

  VecXd expected(6);
  expected << distortion_radial, focal, 1.0, principal_point;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, SimpleRadialReturnCorrectValues) {
  geometry::Camera camera = geometry::Camera::CreateSimpleRadialCamera(
      focal, 1.0, principal_point, distortion_radial[0]);
  const auto values = camera.GetParametersValues();

  VecXd expected(5);
  expected << distortion_radial[0], focal, 1.0, principal_point;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, BrownReturnCorrectValues) {
  geometry::Camera camera = geometry::Camera::CreateBrownCamera(
      focal, 1.0, principal_point, distortion_brown);
  const auto values = camera.GetParametersValues();

  VecXd expected(9);
  expected << distortion_brown, focal, 1.0, principal_point;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, DualReturnCorrectValues) {
  geometry::Camera camera = geometry::Camera::Camera::CreateDualCamera(
      0.5, focal, distortion[0], distortion[1]);
  const auto values = camera.GetParametersValues();

  VecXd expected(4);
  expected << 0.5, distortion[0], distortion[1], focal;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, PerspectiveReturnCorrectK) {
  geometry::Camera camera = geometry::Camera::CreatePerspectiveCamera(
      focal, distortion[0], distortion[1]);

  Mat3d expected = Mat3d::Identity();
  expected(0, 0) = expected(1, 1) = focal;

  ASSERT_TRUE(expected.isApprox(camera.GetProjectionMatrix()));
}

TEST_F(CameraFixture, CanSetParameter) {
  geometry::Camera camera = geometry::Camera::CreatePerspectiveCamera(
      focal, distortion[0], distortion[1]);

  const double new_focal = 0.1;
  camera.SetParameterValue(geometry::Camera::Parameters::Focal, new_focal);
  const auto values = camera.GetParametersValues();

  VecXd expected(3);
  expected << distortion[0], distortion[1], new_focal;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, DoSetParameter) {
  geometry::Camera camera = geometry::Camera::CreatePerspectiveCamera(
      focal, distortion[0], distortion[1]);
  const auto values = camera.GetParametersValues();
  EXPECT_NO_THROW(
      camera.SetParameterValue(geometry::Camera::Parameters::Focal, 0.1));
}

TEST_F(CameraFixture, DoesntSetParameter) {
  geometry::Camera camera = geometry::Camera::CreatePerspectiveCamera(
      focal, distortion[0], distortion[1]);
  const auto values = camera.GetParametersValues();
  EXPECT_ANY_THROW(
      camera.SetParameterValue(geometry::Camera::Parameters::AspectRatio, 0.0));
}

TEST_F(CameraFixture, PerspectiveReturnCorrectKScaled) {
  geometry::Camera camera = geometry::Camera::CreatePerspectiveCamera(
      focal, distortion[0], distortion[1]);

  Mat3d expected = Mat3d::Identity();
  expected(0, 0) = expected(1, 1) = focal * std::max(pixel_width, pixel_height);
  expected(0, 2) = pixel_width * 0.5;
  expected(1, 2) = pixel_height * 0.5;

  ASSERT_TRUE(expected.isApprox(
      camera.GetProjectionMatrixScaled(pixel_width, pixel_height)));
}

TEST_F(CameraFixture, BrownReturnCorrectK) {
  geometry::Camera camera = geometry::Camera::CreateBrownCamera(
      focal, new_ar, principal_point, distortion_brown);

  Mat3d expected = Mat3d::Identity();
  expected(0, 0) = focal;
  expected(1, 1) = new_ar * focal;
  expected(0, 2) = principal_point(0);
  expected(1, 2) = principal_point(1);

  ASSERT_TRUE(expected.isApprox(camera.GetProjectionMatrix()));
}

TEST_F(CameraFixture, BrownReturnCorrectKScaled) {
  geometry::Camera camera = geometry::Camera::CreateBrownCamera(
      focal, new_ar, principal_point, distortion_brown);

  Mat3d expected = Mat3d::Identity();
  const auto normalizer = std::max(pixel_width, pixel_height);
  expected(0, 0) = focal * normalizer;
  expected(1, 1) = (focal * new_ar) * normalizer;
  expected(0, 2) = principal_point(0) * normalizer + pixel_width * 0.5;
  expected(1, 2) = principal_point(1) * normalizer + pixel_height * 0.5;

  ASSERT_TRUE(expected.isApprox(
      camera.GetProjectionMatrixScaled(pixel_width, pixel_height)));
}

TEST_F(CameraFixture, FisheyeOpencvAsFisheye62) {
  Eigen::Matrix<double, 8, 1> dist_62;
  dist_62 << distortion_fisheye[0], distortion_fisheye[1],
      distortion_fisheye[2], distortion_fisheye[3], 0, 0, 0, 0;
  geometry::Camera cam62 = geometry::Camera::CreateFisheye62Camera(
      focal, 1.0, principal_point, dist_62);
  geometry::Camera camcv = geometry::Camera::CreateFisheyeOpencvCamera(
      focal, 1.0, principal_point, distortion_fisheye);
  const MatX3d bear1 = cam62.BearingsMany(pixels);
  const MatX3d bear2 = camcv.BearingsMany(pixels);
  ASSERT_TRUE(bear1.isApprox(bear2, 1e-6));
  const auto proj1 = cam62.ProjectMany(bear1);
  const auto proj2 = camcv.ProjectMany(bear2);
  ASSERT_TRUE(proj1.isApprox(proj2, 1e-6));
}

TEST_F(CameraFixture, SimpleRadialAsRadial) {
  const double k1 = distortion_radial[0];
  geometry::Camera cam_simple = geometry::Camera::CreateSimpleRadialCamera(
      focal, 1.0, principal_point, k1);
  geometry::Camera cam_radial = geometry::Camera::CreateRadialCamera(
      focal, 1.0, principal_point, Vec2d(k1, 0));
  const MatX3d bear1 = cam_simple.BearingsMany(pixels);
  const MatX3d bear2 = cam_radial.BearingsMany(pixels);
  ASSERT_TRUE(bear1.isApprox(bear2, 1e-6));
  const auto proj1 = cam_simple.ProjectMany(bear1);
  const auto proj2 = cam_radial.ProjectMany(bear2);
  ASSERT_TRUE(proj1.isApprox(proj2, 1e-6));
}

TEST(Camera, TestPixelNormalizedCoordinatesConversion) {
  constexpr auto width{640}, height{480};
  auto cam = geometry::Camera::CreatePerspectiveCamera(1.0, 0.0, 0.0);
  cam.width = width;
  cam.height = height;
  const auto inv_normalizer = 1.0 / std::max(width, height);
  const Vec2d px_coord_def(200, 100);
  const Vec2d norm_coord_comp = cam.PixelToNormalizedCoordinates(px_coord_def);
  ASSERT_EQ(norm_coord_comp[0],
            (px_coord_def[0] - (width - 1) / 2.0) * inv_normalizer);
  ASSERT_EQ(norm_coord_comp[1],
            (px_coord_def[1] - (height - 1) / 2.0) * inv_normalizer);
  const Vec2d px_coord_comp = cam.NormalizedToPixelCoordinates(norm_coord_comp);
  ASSERT_EQ(px_coord_comp, px_coord_def);

  const Vec2d norm_coord_static =
      geometry::Camera::PixelToNormalizedCoordinates(px_coord_def, width,
                                                     height);
  ASSERT_EQ(norm_coord_comp[0], norm_coord_static[0]);
  ASSERT_EQ(norm_coord_comp[1], norm_coord_static[1]);
  const Vec2d px_coord_static = geometry::Camera::NormalizedToPixelCoordinates(
      norm_coord_static, width, height);
  ASSERT_EQ(px_coord_comp[0], px_coord_static[0]);
  ASSERT_EQ(px_coord_comp[1], px_coord_static[1]);
}

TEST(Camera, TestCameraProjectionTypes) {
  ASSERT_EQ(
      geometry::Camera::GetProjectionString(geometry::ProjectionType::BROWN),
      "brown");
  ASSERT_EQ(geometry::Camera::GetProjectionString(
                geometry::ProjectionType::PERSPECTIVE),
            "perspective");
  ASSERT_EQ(
      geometry::Camera::GetProjectionString(geometry::ProjectionType::DUAL),
      "dual");
  ASSERT_EQ(
      geometry::Camera::GetProjectionString(geometry::ProjectionType::FISHEYE),
      "fisheye");
  ASSERT_EQ(geometry::Camera::GetProjectionString(
                geometry::ProjectionType::FISHEYE_OPENCV),
            "fisheye_opencv");
  ASSERT_EQ(geometry::Camera::GetProjectionString(
                geometry::ProjectionType::FISHEYE62),
            "fisheye62");
  ASSERT_EQ(geometry::Camera::GetProjectionString(
                geometry::ProjectionType::SPHERICAL),
            "spherical");
  ASSERT_EQ(
      geometry::Camera::GetProjectionString(geometry::ProjectionType::RADIAL),
      "radial");
  ASSERT_EQ(geometry::Camera::GetProjectionString(
                geometry::ProjectionType::SIMPLE_RADIAL),
            "simple_radial");

  // One test to ensure that it is passed correctly from the cam to the static
  // method
  ASSERT_EQ(
      geometry::Camera::CreatePerspectiveCamera(0, 0, 0).GetProjectionString(),
      "perspective");
}
