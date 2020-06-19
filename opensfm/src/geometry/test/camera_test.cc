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
  void RunJacobianEval(const Camera& camera, ProjectionType projection,
                       MAT* jacobian) {
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

  Eigen::Matrix<double, 2, 6, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, ProjectionType::PERSPECTIVE, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraFixture, ComputeFisheyeAnalyticalDerivatives){
  const Camera camera = Camera::CreateFisheyeCamera(focal, -0.1, 0.01);
  
  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 6, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, ProjectionType::FISHEYE, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraFixture, ComputeBrownAnalyticalDerivatives){
  const Camera camera =
      Camera::CreateBrownCamera(focal, new_ar, principal_point, distortion);

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 12, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, ProjectionType::BROWN, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraFixture, ComputeSphericalAnalyticalDerivatives){
  const Camera camera = Camera::CreateSphericalCamera();

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3;

  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, ProjectionType::SPHERICAL, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraFixture, ComputeDualAnalyticalDerivatives){
  const Camera camera = Camera::Camera::CreateDualCamera(0.5, focal, distortion[0], distortion[1]);

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, ProjectionType::DUAL, &jacobian);
  CheckJacobian(jacobian, size_params);
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

  struct F1 : public CameraFunctor<1, 1, 1>{
    template <class T>
    static void Apply(const T* in, const T* p, T* out) {
      out[0] = p[0] * exp(in[0]);
    }
    template <class T, bool COMP_PARAM>
    static void ForwardDerivatives(const T* in, const T* p,
                                   T* out, T* jacobian) {
      jacobian[0] = p[0] * exp(in[0]);
      if (COMP_PARAM) {
        jacobian[1] = exp(in[0]);
      }
      Apply(in, p, out);
    }
  };
  struct F2 : public CameraFunctor<1, 2, 1>{
    template <class T>
    static void Apply(const T* in, const T* p, T* out) {
      out[0] = p[0] - p[1] * log(in[0]);
    }
    template <class T, bool COMP_PARAM>
    static void ForwardDerivatives(const T* in, const T* p,
                                   T* out, T* jacobian) {
      jacobian[0] = -p[1]/in[0];
      if (COMP_PARAM) {
        jacobian[1] = T(1.0);
        jacobian[2] = -log(in[0]);
      }
      Apply(in, p, out);
    }
  };
  struct F3 : public CameraFunctor<1, 1, 1>{
    template <class T>
    static void Apply(const T* in, const T* p, T* out) {
      out[0] = p[0] * in[0] * in[0];
    }
    template <class T, bool COMP_PARAM>
    static void ForwardDerivatives(const T* in, const T* p,
                                   T* out, T* jacobian) {
      jacobian[0] = T(2.0) * p[0] * in[0];
      if (COMP_PARAM) {
        jacobian[1] = in[0] * in[0];
      }
      Apply(in, p, out);
    }
  };
  struct F4 : public CameraFunctor<1, 2, 1>{
    template <class T>
    static void Apply(const T* in, const T* p, T* out) {
      out[0] = p[0] / in[0] - p[1];
    }
    template <class T, bool COMP_PARAM>
    static void ForwardDerivatives(const T* in, const T* p,
                                   T* out, T* jacobian) {
      jacobian[0] = -p[0] / (in[0] * in[0] );
      if (COMP_PARAM) {
        jacobian[1] = T(1.0)/in[0];
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

  const double expected = 3.0 * exp( 1.0 - 2.0 * log( 3.0 * std::pow(4.0/in[0] - 2.0, 2) ) );
  ASSERT_NEAR(expected, evaluated, 1e-20);
}

TEST_F(FunctionFixture, EvaluatesDerivativesCorrectly) {
  double in[] = {1.0};

  /* Parameters needs to be stored in the order of evaluation :
   * params f4 |params f3 | params f2 | params f1 */
  double parameters[] = {4.0, 2.0, 3.0, 1.0, 2.0, 3.0};
  double evaluated = 0;
  double jacobian[7];
  ComposeForwardDerivatives<double, F1, F2, F3, F4>(in, parameters, &evaluated, &jacobian[0]);

  const double expected = 3.0 * exp( 1.0 - 2.0 * log( 3.0 * std::pow(4.0/in[0] - 2.0, 2) ) );
  ASSERT_NEAR(expected, evaluated, 1e-20);

  /* Jacobian ordering : d_input | d_parameters */
  typedef Eigen::AutoDiffScalar<Eigen::VectorXd> AScalar;
  VecX<AScalar> eval_adiff(1);
  eval_adiff(0).value() = in[0];
  eval_adiff(0).derivatives() = Eigen::VectorXd::Unit(7, 0);
  VecX<AScalar> parameters_adiff(6);
  for (int i = 0; i < 6; ++i) {
    parameters_adiff[i].value() = parameters[i];
    parameters_adiff[i].derivatives() = Eigen::VectorXd::Unit(7, i+1);
  }

  AScalar evaluated_addif;
  ComposeFunctions<AScalar, F1, F2, F3, F4>(
      eval_adiff.data(), parameters_adiff.data(), &evaluated_addif);
  for(int i = 0; i < 7; ++i){
    ASSERT_NEAR(evaluated_addif.derivatives()(i), jacobian[i], 1e-20);
  }
}