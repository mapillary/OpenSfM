#include <random>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

#include <geometry/camera.h>
#include <ceres/jet.h>

#include <chrono> 
using namespace std::chrono; 

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

    for (int i = 0; i < 3; ++i) {
      point_adiff[i].value() = point[i] = (i + 1) / 10.0;
    }
  }

  double ComputeError(const Eigen::MatrixX2d& other) {
    double error = 0;
    for (int i = 0; i < pixels_count; ++i) {
      error += (other.row(i) - pixels.row(i)).norm();
    }
    return error/pixels_count;
  }

  template <class MAT>
  void CheckJacobian(const MAT& jacobian, int size_params) {
    const double eps = 1e-12;
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < size_params; ++j) {
        ASSERT_NEAR(projection_expected[i].derivatives()(j), jacobian(i, j), eps);
      }
      ASSERT_NEAR(projection_expected[i].value(), projected[i], eps);
    }
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

  double point[3];
  typedef Eigen::AutoDiffScalar<Eigen::VectorXd> AScalar;
  AScalar point_adiff[3];
  VecX<AScalar> camera_adiff;

  AScalar projection_expected[2];
  double projected[2];
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

TEST_F(CameraFixture, SphericalReturnCorrectTypes){
  Camera camera = Camera::CreateSphericalCamera();
  const auto types = camera.GetParametersTypes();
  ASSERT_EQ(1, types.size());
  ASSERT_EQ(Camera::Parameters::None, types[0]);
}

TEST_F(CameraFixture, PerspectiveReturnCorrectTypes){
  Camera camera = Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);
  const auto types = camera.GetParametersTypes();
  const auto expected = std::vector<Camera::Parameters>(
      {Camera::Parameters::K1, Camera::Parameters::K2,
       Camera::Parameters::Focal});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, FisheyeReturnCorrectTypes) {
  Camera camera = Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);
  const auto types = camera.GetParametersTypes();
  const auto expected = std::vector<Camera::Parameters>(
      {Camera::Parameters::K1, Camera::Parameters::K2,
       Camera::Parameters::Focal});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, BrownReturnCorrectTypes){
  Camera camera = Camera::CreateBrownCamera(focal, 1.0, principal_point, distortion);
  const auto types = camera.GetParametersTypes();

  const auto expected = std::vector<Camera::Parameters>(
      {Camera::Parameters::K1, Camera::Parameters::K2, Camera::Parameters::K3,
       Camera::Parameters::P1, Camera::Parameters::P2,
       Camera::Parameters::Focal, Camera::Parameters::AspectRatio,
       Camera::Parameters::Cx, Camera::Parameters::Cy});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, DualReturnCorrectTypes){
  Camera camera = Camera::Camera::CreateDualCamera(0.5, focal, distortion[0], distortion[1]);
  const auto types = camera.GetParametersTypes();
  const auto expected = std::vector<Camera::Parameters>(
      {Camera::Parameters::Transition, Camera::Parameters::K1,
       Camera::Parameters::K2, Camera::Parameters::Focal});
  ASSERT_THAT(expected, ::testing::ContainerEq(types));
}

TEST_F(CameraFixture, PerspectiveReturnCorrectValues){
  Camera camera = Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);
  const auto values = camera.GetParametersValues();

  Eigen::VectorXd expected(3);
  expected << distortion[0], distortion[1], focal;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, FisheyeReturnCorrectValues) {
  Camera camera = Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);
  const auto values = camera.GetParametersValues();

  Eigen::VectorXd expected(3);
  expected << distortion[0], distortion[1], focal;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, BrownReturnCorrectValues){
  Camera camera = Camera::CreateBrownCamera(focal, 1.0, principal_point, distortion);
  const auto values = camera.GetParametersValues();

  Eigen::VectorXd expected(9);
  expected << distortion, focal, 1.0, principal_point;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, DualReturnCorrectValues){
  Camera camera = Camera::Camera::CreateDualCamera(0.5, focal, distortion[0], distortion[1]);
  const auto values = camera.GetParametersValues();

  Eigen::VectorXd expected(4);
  expected << 0.5, distortion[0], distortion[1], focal;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, PerspectiveReturnCorrectK){
  Camera camera = Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);

  Eigen::Matrix3d expected = Eigen::Matrix3d::Identity();
  expected(0, 0) = expected(1, 1) = focal;
  ASSERT_EQ(expected, camera.GetProjectionMatrix());
}

TEST_F(CameraFixture, CanSetParameter){
  Camera camera = Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);

  const double new_focal = 0.1;
  camera.SetParameterValue(Camera::Parameters::Focal, new_focal);
  const auto values = camera.GetParametersValues();

  Eigen::VectorXd expected(3);
  expected << distortion[0], distortion[1], new_focal;
  ASSERT_EQ(expected, values);
}

TEST_F(CameraFixture, DoSetParameter){
  Camera camera = Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);
  const auto values = camera.GetParametersValues();
  EXPECT_NO_THROW(camera.SetParameterValue(Camera::Parameters::Focal, 0.1));
}

TEST_F(CameraFixture, DoesntSetParameter){
  Camera camera = Camera::CreatePerspectiveCamera(focal, distortion[0], distortion[1]);
  const auto values = camera.GetParametersValues();
  EXPECT_ANY_THROW(camera.SetParameterValue(Camera::Parameters::AspectRatio, 0.0));
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

TEST_F(CameraFixture, ComputePerspectiveAnalyticalDerivatives){
  const Camera camera = Camera::CreatePerspectiveCamera(focal, -0.1, 0.01);
  
  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  // Prepare Eigen's Autodiff structures
  for (int i = 0; i < 3; ++i) {
    point_adiff[i].derivatives() = Eigen::VectorXd::Unit(size_params, i);
  }
  camera_adiff.resize(camera_params.size());
  for (int i = 0; i < camera_params.size(); ++i) {
    camera_adiff(i).value() = camera_params(i);
    camera_adiff(i).derivatives() = Eigen::VectorXd::Unit(size_params, 3 + i);
  }

  // Run project with Autodiff types to get expected jacobian
  Dispatch<ProjectFunction>(ProjectionType::PERSPECTIVE, point_adiff,
                            camera_adiff.data(), projection_expected);

  // Analytical derivatives
  Eigen::Matrix<double, 2, 6, Eigen::RowMajor> jacobian;
  Dispatch<ProjectDerivativesFunction>(ProjectionType::PERSPECTIVE, point,
                                       camera_params.data(), projected,
                                       jacobian.data());
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraFixture, ComputeFisheyeAnalyticalDerivatives){
  const Camera camera = Camera::CreateFisheyeCamera(focal, -0.1, 0.01);
  
  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  // Prepare Eigen's Autodiff structures
  for (int i = 0; i < 3; ++i) {
    point_adiff[i].derivatives() = Eigen::VectorXd::Unit(size_params, i);
  }
  camera_adiff.resize(camera_params.size());
  for (int i = 0; i < camera_params.size(); ++i) {
    camera_adiff(i).value() = camera_params(i);
    camera_adiff(i).derivatives() = Eigen::VectorXd::Unit(size_params, 3 + i);
  }

  // Run project with Autodiff types to get expected jacobian
  Dispatch<ProjectFunction>(ProjectionType::FISHEYE, point_adiff,
                            camera_adiff.data(), projection_expected);

  // Analytical derivatives
  Eigen::Matrix<double, 2, 6, Eigen::RowMajor> jacobian;
  Dispatch<ProjectDerivativesFunction>(ProjectionType::FISHEYE, point,
                                       camera_params.data(), projected,
                                       jacobian.data());
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraFixture, ComputeBrownAnalyticalDerivatives){
  const Camera camera = Camera::CreateBrownCamera(focal, new_ar, principal_point, distortion);
  
  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  // Prepare Eigen's Autodiff structures
  for (int i = 0; i < 3; ++i) {
    point_adiff[i].derivatives() = Eigen::VectorXd::Unit(size_params, i);
  }
  camera_adiff.resize(camera_params.size());
  for (int i = 0; i < camera_params.size(); ++i) {
    camera_adiff(i).value() = camera_params(i);
    camera_adiff(i).derivatives() = Eigen::VectorXd::Unit(size_params, 3 + i);
  }

  // Run project with Autodiff types to get expected jacobian
  Dispatch<ProjectFunction>(ProjectionType::BROWN, point_adiff,
                            camera_adiff.data(), projection_expected);

  // Analytical derivatives
  Eigen::Matrix<double, 2, 12, Eigen::RowMajor> jacobian;
  Dispatch<ProjectDerivativesFunction>(ProjectionType::BROWN, point,
                                       camera_params.data(), projected,
                                       jacobian.data());
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraFixture, PerfTest){
  const Camera camera = Camera::CreateBrownCamera(focal, new_ar, principal_point, distortion);
  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  typedef Eigen::AutoDiffScalar<Eigen::VectorXd> AScalar;

  auto start_ceres = high_resolution_clock::now(); 
  double point_d_ceres[3];

  double a = 0.;
  ceres::Jet<double, 12> projected_ceres[2];
  const int count = 100000;
  for (int k = 0; k < count; ++k) {
    ceres::Jet<double, 12> point_ceres[3];
    for(int i = 0; i < 3; ++i){
      point_ceres[i].a = (i+1)/10.0;
      point_ceres[i].v = Eigen::VectorXd::Unit(size_params, i);
      point_d_ceres[i] = (i+1)/10.0;
    }

    VecX<ceres::Jet<double, 12>> camera_adiff_ceres(camera_params.size());
    for(int i = 0; i < camera_params.size(); ++i){
      camera_adiff_ceres(i).a = camera_params(i);
      camera_adiff_ceres(i).v = Eigen::VectorXd::Unit(size_params, 3+i);
    }
  
    Dispatch<ProjectFunction>(ProjectionType::BROWN, point_ceres,
                              camera_adiff_ceres.data(), projected_ceres);
    for(int i = 0; i < 2; ++i){
      for(int j = 0; j < size_params; ++j){
        a += projected_ceres[i].a;
        a += projected_ceres[i].v(j);
      }
    }
  }
  auto stop_ceres = high_resolution_clock::now(); 
  auto duration_ceres = duration_cast<microseconds>(stop_ceres - start_ceres); 
  std::cout << "CERES AUTODIFF TIME " << duration_ceres.count() << std::endl;
  for(int i = 0; i < 2; ++i){
    for(int j = 0; j < size_params; ++j){
      std::cout << projected_ceres[i].v(j) << " ";
    }
    std::cout << std::endl;
  }

  auto start_auto = high_resolution_clock::now(); 
  

  AScalar projected_eigen[2];
  for (int k = 0; k < count; ++k) {
    AScalar point[3];
    for(int i = 0; i < 3; ++i){
      point[i].value() = (i+1)/10.0;
      point[i].derivatives() = Eigen::VectorXd::Unit(size_params, i);
    }

    VecX<AScalar> camera_adiff(camera_params.size());
    for(int i = 0; i < camera_params.size(); ++i){
      camera_adiff(i).value() = camera_params(i);
      camera_adiff(i).derivatives() = Eigen::VectorXd::Unit(size_params, 3+i);
    }

    Dispatch<ProjectFunction>(ProjectionType::BROWN, point,
                              camera_adiff.data(), projected_eigen);
    for(int i = 0; i < 2; ++i){
      for(int j = 0; j < size_params; ++j){
        a += camera_adiff(i).value();
        a += camera_adiff(i).derivatives()(j);
      }
    }
  }
  auto stop_auto = high_resolution_clock::now(); 
  auto duration_auto = duration_cast<microseconds>(stop_auto - start_auto); 
  std::cout << "EIGEN AUTODIFF TIME " << duration_auto.count() << std::endl; 
  for(int i = 0; i < 2; ++i){
    for(int j = 0; j < size_params; ++j){
      std::cout << projected_eigen[i].derivatives()(j) << " ";
    }
    std::cout << std::endl;
  }

  auto start_ana = high_resolution_clock::now(); 
  double projected_d[2];
  Eigen::Matrix<double, 2, 12, Eigen::RowMajor> jacobian;
  for (int k = 0; k < count; ++k) {
    double point_d[3];
    AScalar point[3];
    for(int i = 0; i < 3; ++i){
      point_d[i] = (i+1)/10.0;
    }

    
    Dispatch<ProjectDerivativesFunction>(ProjectionType::BROWN, point_d,
                                         camera_params.data(), projected_d,
                                         jacobian.data());
    for(int i = 0; i < jacobian.rows(); ++i){
      for(int j = 0; j < jacobian.cols(); ++j){
        a += jacobian(i, j);
      }
    }
  }
  auto stop_ana = high_resolution_clock::now(); 
  auto duration_ana = duration_cast<microseconds>(stop_ana - start_ana); 
  std::cout << "ANALYTICAL TIME " << duration_ana.count() << std::endl; 
  for(int i = 0; i < jacobian.rows(); ++i){
    for(int j = 0; j < jacobian.cols(); ++j){
      std::cout << jacobian(i, j) << " ";
    }
    std::cout << std::endl;
  }
  std::cout << a << std::endl;
}