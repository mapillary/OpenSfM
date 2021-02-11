#include <ceres/jet.h>
#include <ceres/rotation.h>
#include <geometry/camera.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <random>
#include <unsupported/Eigen/AutoDiff>

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

    for (int i = 0; i < 3; ++i) {
      point_adiff[i].value() = point[i] = (i + 1) / 10.0;
    }
  }

  double ComputeError(const MatX2d& other) {
    double error = 0;
    for (int i = 0; i < pixels_count; ++i) {
      error += (other.row(i) - pixels.row(i)).norm();
    }
    return error / pixels_count;
  }

  template <class MAT>
  void RunJacobianEval(const Camera& camera, ProjectionType projection,
                       MAT* jacobian) {
    const VecXd camera_params = camera.GetParametersValues();
    const int size_params = 3 + camera_params.size();

    // Prepare Eigen's Autodiff structures
    for (int i = 0; i < 3; ++i) {
      point_adiff[i].derivatives() = VecXd::Unit(size_params, i);
    }
    camera_adiff.resize(camera_params.size());
    for (int i = 0; i < camera_params.size(); ++i) {
      camera_adiff(i).value() = camera_params(i);
      camera_adiff(i).derivatives() = VecXd::Unit(size_params, 3 + i);
    }

    // Run project with Autodiff types to get expected jacobian
    Dispatch<ProjectFunction>(projection, point_adiff, camera_adiff.data(),
                              projection_expected);

    // Analytical derivatives
    Dispatch<ProjectDerivativesFunction>(
        projection, point, camera_params.data(), projected, jacobian->data());
  }

  template <class MAT>
  void CheckJacobian(const MAT& jacobian, int size_params) {
    const double eps = 1e-12;
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < size_params; ++j) {
        ASSERT_NEAR(projection_expected[i].derivatives()(j), jacobian(i, j),
                    eps);
      }
      ASSERT_NEAR(projection_expected[i].value(), projected[i], eps);
    }
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
  typedef Eigen::AutoDiffScalar<VecXd> AScalar;
  AScalar point_adiff[3];
  VecX<AScalar> camera_adiff;

  AScalar projection_expected[2];
  double projected[2];
};

TEST_F(CameraFixture, PerspectiveIsConsistent) {
  Camera camera =
      Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, BrownIsConsistent) {
  Camera camera =
      Camera::CreateBrownCamera(focal, 1.0, principal_point, distortion_brown);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, FisheyeIsConsistent) {
  Camera camera =
      Camera::CreateFisheyeCamera(focal, distortion[0], distortion[1]);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, FisheyeIsConsistentLargeFov) {
  double short_focal = 0.3;
  Camera camera = Camera::CreateFisheyeCamera(short_focal, 0.001, 0.001);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, FisheyeOpencvIsConsistent) {
  Camera camera = Camera::CreateFisheyeOpencvCamera(focal, 1.0, principal_point,
                                                    distortion_fisheye);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, Fisheye62IsConsistent) {
  Camera camera = Camera::CreateFisheye62Camera(focal, 1.0, principal_point,
                                                distortion_fisheye62);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, DualIsConsistent) {
  Camera camera =
      Camera::CreateDualCamera(0.5, focal, distortion[0], distortion[1]);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, SphericalIsConsistent) {
  Camera camera = Camera::CreateSphericalCamera();
  const auto projected = camera.Project(camera.Bearing(Vec2d(0.2, 0.2)));
  ASSERT_EQ(projected(0), 0.2);
  ASSERT_EQ(projected(1), 0.2);
}

TEST_F(CameraFixture, RadialIsConsistent) {
  Camera camera = Camera::CreateRadialCamera(focal, 1.0, principal_point,
                                             distortion_radial);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, SimpleRadialIsConsistent) {
  Camera camera = Camera::CreateSimpleRadialCamera(focal, 1.0, principal_point,
                                                   distortion_radial[0]);
  const auto projected = camera.ProjectMany(camera.BearingsMany(pixels));
  ASSERT_LT(ComputeError(projected), 2e-7);
}

TEST_F(CameraFixture, SphericalReturnCorrectTypes) {
  Camera camera = Camera::CreateSphericalCamera();
  const auto types = camera.GetParametersTypes();
  ASSERT_EQ(1, types.size());
  ASSERT_EQ(Camera::Parameters::None, types[0]);
}

TEST_F(CameraFixture, PerspectiveReturnCorrectTypes) {
  Camera camera =
      Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);
  const auto types = camera.GetParametersTypes();
  const auto expected = std::vector<Camera::Parameters>(
      {Camera::Parameters::K1, Camera::Parameters::K2,
       Camera::Parameters::Focal});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, FisheyeReturnCorrectTypes) {
  Camera camera =
      Camera::CreateFisheyeCamera(focal, distortion[0], distortion[1]);
  const auto types = camera.GetParametersTypes();
  const auto expected = std::vector<Camera::Parameters>(
      {Camera::Parameters::K1, Camera::Parameters::K2,
       Camera::Parameters::Focal});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, FisheyeOpencvReturnCorrectTypes) {
  Camera camera = Camera::CreateFisheyeOpencvCamera(focal, 1.0, principal_point,
                                                    distortion_fisheye);
  const auto types = camera.GetParametersTypes();
  const auto expected = std::vector<Camera::Parameters>(
      {Camera::Parameters::K1, Camera::Parameters::K2, Camera::Parameters::K3,
       Camera::Parameters::K4, Camera::Parameters::Focal,
       Camera::Parameters::AspectRatio, Camera::Parameters::Cx,
       Camera::Parameters::Cy});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, Fisheye62ReturnCorrectTypes) {
  Camera camera = Camera::CreateFisheye62Camera(focal, 1.0, principal_point,
                                                distortion_fisheye62);
  const auto types = camera.GetParametersTypes();
  const auto expected = std::vector<Camera::Parameters>(
      {Camera::Parameters::K1, Camera::Parameters::K2, Camera::Parameters::K3,
       Camera::Parameters::K4, Camera::Parameters::K5, Camera::Parameters::K6,
       Camera::Parameters::P1, Camera::Parameters::P2,
       Camera::Parameters::Focal, Camera::Parameters::AspectRatio,
       Camera::Parameters::Cx, Camera::Parameters::Cy});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, RadialReturnCorrectTypes) {
  Camera camera = Camera::CreateRadialCamera(focal, 1.0, principal_point,
                                             distortion_radial);
  const auto types = camera.GetParametersTypes();
  const auto expected = std::vector<Camera::Parameters>(
      {Camera::Parameters::K1, Camera::Parameters::K2,
       Camera::Parameters::Focal, Camera::Parameters::AspectRatio,
       Camera::Parameters::Cx, Camera::Parameters::Cy});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, SimpleRadialReturnCorrectTypes) {
  Camera camera = Camera::CreateSimpleRadialCamera(focal, 1.0, principal_point,
                                                   distortion_radial[0]);
  const auto types = camera.GetParametersTypes();
  const auto expected = std::vector<Camera::Parameters>(
      {Camera::Parameters::K1, Camera::Parameters::Focal,
       Camera::Parameters::AspectRatio, Camera::Parameters::Cx,
       Camera::Parameters::Cy});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, BrownReturnCorrectTypes) {
  Camera camera =
      Camera::CreateBrownCamera(focal, 1.0, principal_point, distortion_brown);
  const auto types = camera.GetParametersTypes();

  const auto expected = std::vector<Camera::Parameters>(
      {Camera::Parameters::K1, Camera::Parameters::K2, Camera::Parameters::K3,
       Camera::Parameters::P1, Camera::Parameters::P2,
       Camera::Parameters::Focal, Camera::Parameters::AspectRatio,
       Camera::Parameters::Cx, Camera::Parameters::Cy});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, DualReturnCorrectTypes) {
  Camera camera = Camera::Camera::CreateDualCamera(0.5, focal, distortion[0],
                                                   distortion[1]);
  const auto types = camera.GetParametersTypes();
  const auto expected = std::vector<Camera::Parameters>(
      {Camera::Parameters::Transition, Camera::Parameters::K1,
       Camera::Parameters::K2, Camera::Parameters::Focal});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, PerspectiveReturnCorrectValues) {
  Camera camera =
      Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);
  const auto values = camera.GetParametersValues();

  VecXd expected(3);
  expected << distortion[0], distortion[1], focal;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, FisheyeReturnCorrectValues) {
  Camera camera =
      Camera::CreateFisheyeCamera(focal, distortion[0], distortion[1]);
  const auto values = camera.GetParametersValues();

  VecXd expected(3);
  expected << distortion[0], distortion[1], focal;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, FisheyeOpencvReturnCorrectValues) {
  Camera camera = Camera::CreateFisheyeOpencvCamera(focal, 1.0, principal_point,
                                                    distortion_fisheye);
  const auto values = camera.GetParametersValues();

  VecXd expected(8);
  expected << distortion_fisheye, focal, 1.0, principal_point;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, Fisheye62ReturnCorrectValues) {
  Camera camera = Camera::CreateFisheye62Camera(focal, 1.0, principal_point,
                                                distortion_fisheye62);
  const auto values = camera.GetParametersValues();

  VecXd expected(12);
  expected << distortion_fisheye62, focal, 1.0, principal_point;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, RadialReturnCorrectValues) {
  Camera camera = Camera::CreateRadialCamera(focal, 1.0, principal_point,
                                             distortion_radial);
  const auto values = camera.GetParametersValues();

  VecXd expected(6);
  expected << distortion_radial, focal, 1.0, principal_point;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, SimpleRadialReturnCorrectValues) {
  Camera camera = Camera::CreateSimpleRadialCamera(focal, 1.0, principal_point,
                                                   distortion_radial[0]);
  const auto values = camera.GetParametersValues();

  VecXd expected(5);
  expected << distortion_radial[0], focal, 1.0, principal_point;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, BrownReturnCorrectValues) {
  Camera camera =
      Camera::CreateBrownCamera(focal, 1.0, principal_point, distortion_brown);
  const auto values = camera.GetParametersValues();

  VecXd expected(9);
  expected << distortion_brown, focal, 1.0, principal_point;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, DualReturnCorrectValues) {
  Camera camera = Camera::Camera::CreateDualCamera(0.5, focal, distortion[0],
                                                   distortion[1]);
  const auto values = camera.GetParametersValues();

  VecXd expected(4);
  expected << 0.5, distortion[0], distortion[1], focal;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, PerspectiveReturnCorrectK) {
  Camera camera =
      Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);

  Mat3d expected = Mat3d::Identity();
  expected(0, 0) = expected(1, 1) = focal;

  ASSERT_TRUE(expected.isApprox(camera.GetProjectionMatrix()));
}

TEST_F(CameraFixture, CanSetParameter) {
  Camera camera =
      Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);

  const double new_focal = 0.1;
  camera.SetParameterValue(Camera::Parameters::Focal, new_focal);
  const auto values = camera.GetParametersValues();

  VecXd expected(3);
  expected << distortion[0], distortion[1], new_focal;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, DoSetParameter) {
  Camera camera =
      Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);
  const auto values = camera.GetParametersValues();
  EXPECT_NO_THROW(camera.SetParameterValue(Camera::Parameters::Focal, 0.1));
}

TEST_F(CameraFixture, DoesntSetParameter) {
  Camera camera =
      Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);
  const auto values = camera.GetParametersValues();
  EXPECT_ANY_THROW(
      camera.SetParameterValue(Camera::Parameters::AspectRatio, 0.0));
}

TEST_F(CameraFixture, PerspectiveReturnCorrectKScaled) {
  Camera camera =
      Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);

  Mat3d expected = Mat3d::Identity();
  expected(0, 0) = expected(1, 1) = focal * std::max(pixel_width, pixel_height);
  expected(0, 2) = pixel_width * 0.5;
  expected(1, 2) = pixel_height * 0.5;

  ASSERT_TRUE(expected.isApprox(
      camera.GetProjectionMatrixScaled(pixel_width, pixel_height)));
}

TEST_F(CameraFixture, BrownReturnCorrectK) {
  Camera camera = Camera::CreateBrownCamera(focal, new_ar, principal_point,
                                            distortion_brown);

  Mat3d expected = Mat3d::Identity();
  expected(0, 0) = focal;
  expected(1, 1) = new_ar * focal;
  expected(0, 2) = principal_point(0);
  expected(1, 2) = principal_point(1);

  ASSERT_TRUE(expected.isApprox(camera.GetProjectionMatrix()));
}

TEST_F(CameraFixture, BrownReturnCorrectKScaled) {
  Camera camera = Camera::CreateBrownCamera(focal, new_ar, principal_point,
                                            distortion_brown);

  Mat3d expected = Mat3d::Identity();
  const auto normalizer = std::max(pixel_width, pixel_height);
  expected(0, 0) = focal * normalizer;
  expected(1, 1) = (focal * new_ar) * normalizer;
  expected(0, 2) = principal_point(0) * normalizer + pixel_width * 0.5;
  expected(1, 2) = principal_point(1) * normalizer + pixel_height * 0.5;

  ASSERT_TRUE(expected.isApprox(
      camera.GetProjectionMatrixScaled(pixel_width, pixel_height)));
}

TEST_F(CameraFixture, ComputePerspectiveAnalyticalDerivatives) {
  const Camera camera = Camera::CreatePerspectiveCamera(focal, -0.1, 0.01);

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 6, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, ProjectionType::PERSPECTIVE, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraFixture, ComputeFisheyeAnalyticalDerivatives) {
  const Camera camera = Camera::CreateFisheyeCamera(focal, -0.1, 0.01);

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 6, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, ProjectionType::FISHEYE, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraFixture, ComputeFisheyeOpencvAnalyticalDerivatives) {
  const Camera camera = Camera::CreateFisheyeOpencvCamera(
      focal, new_ar, principal_point, distortion_fisheye);

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 11, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, ProjectionType::FISHEYE_OPENCV, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraFixture, ComputeFisheye62AnalyticalDerivatives) {
  const Camera camera = Camera::CreateFisheye62Camera(
      focal, 1.0, principal_point, distortion_fisheye62);

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 15, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, ProjectionType::FISHEYE62, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraFixture, ComputeRadialAnalyticalDerivatives) {
  const Camera camera = Camera::CreateRadialCamera(focal, 1.0, principal_point,
                                                   distortion_radial);

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 9, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, ProjectionType::RADIAL, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraFixture, ComputeSimpleRadialAnalyticalDerivatives) {
  const Camera camera = Camera::CreateSimpleRadialCamera(
      focal, 1.0, principal_point, distortion_radial[0]);

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 8, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, ProjectionType::SIMPLE_RADIAL, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraFixture, ComputeBrownAnalyticalDerivatives) {
  const Camera camera = Camera::CreateBrownCamera(
      focal, new_ar, principal_point, distortion_brown);

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 12, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, ProjectionType::BROWN, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraFixture, ComputeSphericalAnalyticalDerivatives) {
  const Camera camera = Camera::CreateSphericalCamera();

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3;

  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, ProjectionType::SPHERICAL, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraFixture, ComputeDualAnalyticalDerivatives) {
  const Camera camera = Camera::Camera::CreateDualCamera(
      0.5, focal, distortion[0], distortion[1]);

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, ProjectionType::DUAL, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraFixture, FisheyeOpencvAsFisheye62) {
  Eigen::Matrix<double, 8, 1> dist_62;
  dist_62 << distortion_fisheye[0], distortion_fisheye[1],
      distortion_fisheye[2], distortion_fisheye[3], 0, 0, 0, 0;
  Camera cam62 =
      Camera::CreateFisheye62Camera(focal, 1.0, principal_point, dist_62);
  Camera camcv = Camera::CreateFisheyeOpencvCamera(focal, 1.0, principal_point,
                                                   distortion_fisheye);
  const MatX3d bear1 = cam62.BearingsMany(pixels);
  const MatX3d bear2 = camcv.BearingsMany(pixels);
  ASSERT_TRUE(bear1.isApprox(bear2, 1e-6));
  const auto proj1 = cam62.ProjectMany(bear1);
  const auto proj2 = camcv.ProjectMany(bear2);
  ASSERT_TRUE(proj1.isApprox(proj2, 1e-6));
}

TEST_F(CameraFixture, SimpleRadialAsRadial) {
  const double k1 = distortion_radial[0];
  Camera cam_simple =
      Camera::CreateSimpleRadialCamera(focal, 1.0, principal_point, k1);
  Camera cam_radial =
      Camera::CreateRadialCamera(focal, 1.0, principal_point, Vec2d(k1, 0));
  const MatX3d bear1 = cam_simple.BearingsMany(pixels);
  const MatX3d bear2 = cam_radial.BearingsMany(pixels);
  ASSERT_TRUE(bear1.isApprox(bear2, 1e-6));
  const auto proj1 = cam_simple.ProjectMany(bear1);
  const auto proj2 = cam_radial.ProjectMany(bear2);
  ASSERT_TRUE(proj1.isApprox(proj2, 1e-6));
}

class FunctionFixture : public ::testing::Test {
 public:
  /* Evaluates the following function :
   *
   * f = a * exp( b - c * log( d * (e/x - f)^2 ) )
   *
   * Decomposed as follows :
   *
   * f1 = a * exp(x)
   * f2 = b - c * log(x)
   * f3 = d * x^2
   * f4 = e/x - f
   * f = f1(f2(f3(f4))) */

  struct F1 : public CameraFunctor<1, 1, 1> {
    template <class T>
    static void Apply(const T* in, const T* p, T* out) {
      out[0] = p[0] * exp(in[0]);
    }
    template <class T, bool COMP_PARAM>
    static void ForwardDerivatives(const T* in, const T* p, T* out,
                                   T* jacobian) {
      jacobian[0] = p[0] * exp(in[0]);
      if (COMP_PARAM) {
        jacobian[1] = exp(in[0]);
      }
      Apply(in, p, out);
    }
  };
  struct F2 : public CameraFunctor<1, 2, 1> {
    template <class T>
    static void Apply(const T* in, const T* p, T* out) {
      out[0] = p[0] - p[1] * log(in[0]);
    }
    template <class T, bool COMP_PARAM>
    static void ForwardDerivatives(const T* in, const T* p, T* out,
                                   T* jacobian) {
      jacobian[0] = -p[1] / in[0];
      if (COMP_PARAM) {
        jacobian[1] = T(1.0);
        jacobian[2] = -log(in[0]);
      }
      Apply(in, p, out);
    }
  };
  struct F3 : public CameraFunctor<1, 1, 1> {
    template <class T>
    static void Apply(const T* in, const T* p, T* out) {
      out[0] = p[0] * in[0] * in[0];
    }
    template <class T, bool COMP_PARAM>
    static void ForwardDerivatives(const T* in, const T* p, T* out,
                                   T* jacobian) {
      jacobian[0] = T(2.0) * p[0] * in[0];
      if (COMP_PARAM) {
        jacobian[1] = in[0] * in[0];
      }
      Apply(in, p, out);
    }
  };
  struct F4 : public CameraFunctor<1, 2, 1> {
    template <class T>
    static void Apply(const T* in, const T* p, T* out) {
      out[0] = p[0] / in[0] - p[1];
    }
    template <class T, bool COMP_PARAM>
    static void ForwardDerivatives(const T* in, const T* p, T* out,
                                   T* jacobian) {
      jacobian[0] = -p[0] / (in[0] * in[0]);
      if (COMP_PARAM) {
        jacobian[1] = T(1.0) / in[0];
        jacobian[2] = -T(1.0);
      }
      Apply(in, p, out);
    }
  };
};

TEST_F(FunctionFixture, EvaluatesCorrectly) {
  double in[] = {1.0};

  /* Parameters needs to be stored in the order of evaluation :
   * params f4 |params f3 | params f2 | params f1 */
  double parameters[] = {4.0, 2.0, 3.0, 1.0, 2.0, 3.0};
  double evaluated = 0;
  ComposeFunctions<double, F1, F2, F3, F4>(in, parameters, &evaluated);

  const double expected =
      3.0 * exp(1.0 - 2.0 * log(3.0 * std::pow(4.0 / in[0] - 2.0, 2)));
  ASSERT_NEAR(expected, evaluated, 1e-20);
}

TEST_F(FunctionFixture, EvaluatesDerivativesCorrectly) {
  double in[] = {1.0};

  /* Parameters needs to be stored in the order of evaluation :
   * params f4 |params f3 | params f2 | params f1 */
  double parameters[] = {4.0, 2.0, 3.0, 1.0, 2.0, 3.0};
  double evaluated = 0;
  double jacobian[7];
  ComposeForwardDerivatives<double, F1, F2, F3, F4>(in, parameters, &evaluated,
                                                    &jacobian[0]);

  const double expected =
      3.0 * exp(1.0 - 2.0 * log(3.0 * std::pow(4.0 / in[0] - 2.0, 2)));
  ASSERT_NEAR(expected, evaluated, 1e-20);

  /* Jacobian ordering : d_input | d_parameters */
  typedef Eigen::AutoDiffScalar<VecXd> AScalar;
  VecX<AScalar> eval_adiff(1);
  eval_adiff(0).value() = in[0];
  eval_adiff(0).derivatives() = VecXd::Unit(7, 0);
  VecX<AScalar> parameters_adiff(6);
  for (int i = 0; i < 6; ++i) {
    parameters_adiff[i].value() = parameters[i];
    parameters_adiff[i].derivatives() = VecXd::Unit(7, i + 1);
  }

  AScalar evaluated_addif;
  ComposeFunctions<AScalar, F1, F2, F3, F4>(
      eval_adiff.data(), parameters_adiff.data(), &evaluated_addif);
  for (int i = 0; i < 7; ++i) {
    ASSERT_NEAR(evaluated_addif.derivatives()(i), jacobian[i], 1e-20);
  }
}

class PoseFixture : public ::testing::Test {
 public:
  const double point[3] = {1.0, 2.0, 3.0};
  const double rt[6] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
};

TEST_F(PoseFixture, EvaluatesCorrectly) {
  double transformed[3] = {0., 0., 0.};

  /* Parameters order : angle_axis | pose_center */
  Pose::Forward(point, rt, transformed);

  /* Use Ceres as groundtruth */
  const double pt[3] = {
      point[0] - rt[Pose::Tx],
      point[1] - rt[Pose::Ty],
      point[2] - rt[Pose::Tz],
  };
  const double Rt[3] = {-rt[Pose::Rx], -rt[Pose::Ry], -rt[Pose::Rz]};
  double expected[] = {1., 1., 1.};
  ceres::AngleAxisRotatePoint(Rt, pt, expected);

  for (int i = 0; i < 3; ++i) {
    ASSERT_NEAR(expected[i], transformed[i], 1e-15);
  }
}

TEST_F(PoseFixture, EvaluatesDerivativesCorrectly) {
  typedef Eigen::AutoDiffScalar<VecXd> AScalar;

  VecX<AScalar> point_adiff(3);
  constexpr int size = 9;

  /* Autodifferentaied as a reference */
  for (int i = 0; i < 3; ++i) {
    point_adiff(i).value() = point[i];
    point_adiff(i).derivatives() = VecXd::Unit(size, i);
  }
  VecX<AScalar> rt_adiff(6);
  for (int i = 0; i < 6; ++i) {
    rt_adiff[i].value() = rt[i];
    rt_adiff[i].derivatives() = VecXd::Unit(size, 3 + i);
  }
  VecX<AScalar> expected_adiff(3);
  Pose::Forward(point_adiff.data(), rt_adiff.data(), expected_adiff.data());

  /* Analytic version */
  double transformed[] = {0., 0., 0.};
  double jacobian[size * 3];
  Pose::ForwardDerivatives<double, true>(&point[0], &rt[0], &transformed[0],
                                         &jacobian[0]);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < size; ++j) {
      ASSERT_NEAR(expected_adiff(i).derivatives()(j), jacobian[i * size + j],
                  1e-15);
    }
  }
}

TEST(Camera, TestPixelNormalizedCoordinatesConversion) {
  constexpr auto width{640}, height{480};
  auto cam = Camera::CreatePerspectiveCamera(1.0, 0.0, 0.0);
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
      Camera::PixelToNormalizedCoordinates(px_coord_def, width, height);
  ASSERT_EQ(norm_coord_comp[0], norm_coord_static[0]);
  ASSERT_EQ(norm_coord_comp[1], norm_coord_static[1]);
  const Vec2d px_coord_static =
      Camera::NormalizedToPixelCoordinates(norm_coord_static, width, height);
  ASSERT_EQ(px_coord_comp[0], px_coord_static[0]);
  ASSERT_EQ(px_coord_comp[1], px_coord_static[1]);
}

TEST(Camera, TestCameraProjectionTypes) {
  ASSERT_EQ(Camera::GetProjectionString(ProjectionType::BROWN), "brown");
  ASSERT_EQ(Camera::GetProjectionString(ProjectionType::PERSPECTIVE),
            "perspective");
  ASSERT_EQ(Camera::GetProjectionString(ProjectionType::DUAL), "dual");
  ASSERT_EQ(Camera::GetProjectionString(ProjectionType::FISHEYE), "fisheye");
  ASSERT_EQ(Camera::GetProjectionString(ProjectionType::FISHEYE_OPENCV),
            "fisheye_opencv");
  ASSERT_EQ(Camera::GetProjectionString(ProjectionType::FISHEYE62),
            "fisheye62");
  ASSERT_EQ(Camera::GetProjectionString(ProjectionType::SPHERICAL),
            "spherical");
  ASSERT_EQ(Camera::GetProjectionString(ProjectionType::RADIAL), "radial");
  ASSERT_EQ(Camera::GetProjectionString(ProjectionType::SIMPLE_RADIAL),
            "simple_radial");

  // One test to ensure that it is passed correctly from the cam to the static
  // method
  ASSERT_EQ(Camera::CreatePerspectiveCamera(0, 0, 0).GetProjectionString(),
            "perspective");
}
