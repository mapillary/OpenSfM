#include <ceres/jet.h>
#include <ceres/rotation.h>
#include <geometry/camera.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <random>
#include <unsupported/Eigen/AutoDiff>

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

  struct F1 : public geometry::Functor<1, 1, 1> {
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
  struct F2 : public geometry::Functor<1, 2, 1> {
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
  struct F3 : public geometry::Functor<1, 1, 1> {
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
  struct F4 : public geometry::Functor<1, 2, 1> {
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
  geometry::ComposeFunctions<double, F1, F2, F3, F4>(in, parameters,
                                                     &evaluated);

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
  geometry::ComposeForwardDerivatives<double, true, F1, F2, F3, F4>(
      in, parameters, &evaluated, &jacobian[0]);

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
  geometry::ComposeFunctions<AScalar, F1, F2, F3, F4>(
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
  geometry::PoseFunctor::Forward(point, rt, transformed);

  /* Use Ceres as groundtruth */
  const double pt[3] = {
      point[0] - rt[geometry::PoseFunctor::Tx],
      point[1] - rt[geometry::PoseFunctor::Ty],
      point[2] - rt[geometry::PoseFunctor::Tz],
  };
  const double Rt[3] = {-rt[geometry::PoseFunctor::Rx],
                        -rt[geometry::PoseFunctor::Ry],
                        -rt[geometry::PoseFunctor::Rz]};
  double expected[] = {1., 1., 1.};
  ceres::AngleAxisRotatePoint(Rt, pt, expected);

  for (int i = 0; i < 3; ++i) {
    ASSERT_NEAR(expected[i], transformed[i], 1e-15);
  }
}

TEST_F(PoseFixture, EvaluatesAllDerivativesCorrectly) {
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
  geometry::PoseFunctor::Forward(point_adiff.data(), rt_adiff.data(),
                                 expected_adiff.data());

  /* Analytic version */
  double transformed[] = {0., 0., 0.};
  double jacobian[size * 3];
  geometry::PoseFunctor::ForwardDerivatives<double, true>(
      &point[0], &rt[0], &transformed[0], &jacobian[0]);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < size; ++j) {
      ASSERT_NEAR(expected_adiff(i).derivatives()(j), jacobian[i * size + j],
                  1e-15);
    }
  }
}

TEST_F(PoseFixture, EvaluatesPointDerivativesCorrectly) {
  typedef Eigen::AutoDiffScalar<VecXd> AScalar;

  VecX<AScalar> point_adiff(3);

  /* Autodifferentaied as a reference */
  for (int i = 0; i < 3; ++i) {
    point_adiff(i).value() = point[i];
    point_adiff(i).derivatives() = VecXd::Unit(3, i);
  }
  VecX<AScalar> rt_adiff(6);
  for (int i = 0; i < 6; ++i) {
    rt_adiff[i].value() = rt[i];
    rt_adiff[i].derivatives() = Vec3d::Zero();
  }
  VecX<AScalar> expected_adiff(3);
  geometry::PoseFunctor::Forward(point_adiff.data(), rt_adiff.data(),
                                 expected_adiff.data());

  /* Analytic version */
  double transformed[] = {0., 0., 0.};
  double jacobian[9];
  geometry::PoseFunctor::ForwardDerivatives<double, false>(
      &point[0], &rt[0], &transformed[0], &jacobian[0]);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      ASSERT_NEAR(expected_adiff(i).derivatives()(j), jacobian[i * 3 + j],
                  1e-15);
    }
  }
}

class CameraDerivativesFixture : public ::testing::Test {
 public:
  CameraDerivativesFixture() {
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

  template <class MAT>
  void RunJacobianEval(const geometry::Camera& camera,
                       geometry::ProjectionType projection, MAT* jacobian) {
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
    geometry::Dispatch<geometry::ProjectFunction>(
        projection, point_adiff, camera_adiff.data(), projection_expected);

    // Analytical derivatives
    geometry::Dispatch<geometry::ProjectDerivativesFunction>(
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

  const double focal{0.4};
  const double new_ar{0.9};

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

TEST_F(CameraDerivativesFixture, ComputePerspectiveAnalyticalDerivatives) {
  const geometry::Camera camera =
      geometry::Camera::CreatePerspectiveCamera(focal, -0.1, 0.01);

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 6, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, geometry::ProjectionType::PERSPECTIVE, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraDerivativesFixture, ComputeFisheyeAnalyticalDerivatives) {
  const geometry::Camera camera =
      geometry::Camera::CreateFisheyeCamera(focal, -0.1, 0.01);

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 6, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, geometry::ProjectionType::FISHEYE, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraDerivativesFixture, ComputeFisheyeOpencvAnalyticalDerivatives) {
  const geometry::Camera camera = geometry::Camera::CreateFisheyeOpencvCamera(
      focal, new_ar, principal_point, distortion_fisheye);

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 11, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, geometry::ProjectionType::FISHEYE_OPENCV, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraDerivativesFixture, ComputeFisheye62AnalyticalDerivatives) {
  const geometry::Camera camera = geometry::Camera::CreateFisheye62Camera(
      focal, 1.0, principal_point, distortion_fisheye62);

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 15, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, geometry::ProjectionType::FISHEYE62, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraDerivativesFixture, ComputeRadialAnalyticalDerivatives) {
  const geometry::Camera camera = geometry::Camera::CreateRadialCamera(
      focal, 1.0, principal_point, distortion_radial);

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 9, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, geometry::ProjectionType::RADIAL, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraDerivativesFixture, ComputeSimpleRadialAnalyticalDerivatives) {
  const geometry::Camera camera = geometry::Camera::CreateSimpleRadialCamera(
      focal, 1.0, principal_point, distortion_radial[0]);

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 8, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, geometry::ProjectionType::SIMPLE_RADIAL, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraDerivativesFixture, ComputeBrownAnalyticalDerivatives) {
  const geometry::Camera camera = geometry::Camera::CreateBrownCamera(
      focal, new_ar, principal_point, distortion_brown);

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 12, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, geometry::ProjectionType::BROWN, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraDerivativesFixture, ComputeSphericalAnalyticalDerivatives) {
  const geometry::Camera camera = geometry::Camera::CreateSphericalCamera();

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3;

  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, geometry::ProjectionType::SPHERICAL, &jacobian);
  CheckJacobian(jacobian, size_params);
}

TEST_F(CameraDerivativesFixture, ComputeDualAnalyticalDerivatives) {
  const geometry::Camera camera = geometry::Camera::Camera::CreateDualCamera(
      0.5, focal, distortion[0], distortion[1]);

  const VecXd camera_params = camera.GetParametersValues();
  const int size_params = 3 + camera_params.size();

  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> jacobian;
  RunJacobianEval(camera, geometry::ProjectionType::DUAL, &jacobian);
  CheckJacobian(jacobian, size_params);
}
