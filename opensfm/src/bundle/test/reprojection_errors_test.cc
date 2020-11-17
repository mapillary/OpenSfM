#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

#include <bundle/src/projection_errors.h>

class ReprojectionError2DFixture : public ::testing::Test {
 public:
  typedef Eigen::AutoDiffScalar<Eigen::VectorXd> AScalar;
  ReprojectionError2DFixture() { observed << 0.5, 0.5; }

  void SetupADiff(int size, const double* camera, AScalar* camera_adiff) {
    const int total_size = size_point + size_rt + size;
    for (int i = 0; i < size_point; ++i) {
      point_adiff[i].value() = point[i];
      point_adiff[i].derivatives() = VecXd::Unit(total_size, i);
    }
    for (int i = 0; i < size_rt; ++i) {
      rt_adiff[i].value() = rt[i];
      rt_adiff[i].derivatives() = VecXd::Unit(total_size, size_point + i);
    }
    for (int i = 0; i < size; ++i) {
      camera_adiff[i].value() = camera[i];
      camera_adiff[i].derivatives() = VecXd::Unit(total_size, size_point + size_rt + i);
    }
  }

  void CheckJacobians(int size, const double* jac_camera) {
    for (int i = 0; i < size_residual; ++i) {
      for (int j = 0; j < size; ++j) {
        ASSERT_NEAR(residual_adiff[i].derivatives()(size_point + size_rt + j),
                    jac_camera[i * size + j], 1e-15);
      }
    }
    for (int i = 0; i < size_residual; ++i) {
      for (int j = 0; j < size_point; ++j) {
        ASSERT_NEAR(residual_adiff[i].derivatives()(j),
                    jac_point[i * size_point + j], 1e-15);
      }
    }
    for (int i = 0; i < size_residual; ++i) {
      for (int j = 0; j < size_rt; ++j) {
        ASSERT_NEAR(residual_adiff[i].derivatives()(size_point + j),
                    jac_rt[i * size_rt + j], 1e-15);
      }
    }
  }

  template <int N>
  void RunTest(const ProjectionType& type, const double* camera) {
    constexpr int size = N;

    // Autodiff-ed version will be used as reference/expected values
    AScalar camera_adiff[size];
    SetupADiff(size, &camera[0], &camera_adiff[0]);
    ReprojectionError2D autodiff(type, observed, scale);
    autodiff(camera_adiff, rt_adiff, point_adiff, residual_adiff);

    // We test for analytic evaluation
    double jac_camera[size_residual * size];
    const double* params[] = {camera, rt, point};
    double* jacobians[] = {jac_camera, jac_rt, jac_point};
    ReprojectionError2DAnalytic<size> analytic(type, observed, scale);
    analytic.Evaluate(params, residuals, &jacobians[0]);

    // Check
    CheckJacobians(size, jac_camera);
  }

  constexpr static int size_residual = 2;
  constexpr static int size_point = 3;
  constexpr static int size_rt = 6;

  Vec2d observed;
  double scale{0.1};
  const double point[size_point] = {1.0, 2.0, 3.0};
  const double rt[size_rt] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};

  AScalar residual_adiff[size_residual];
  AScalar point_adiff[size_point];
  AScalar rt_adiff[size_rt];

  double residuals[size_residual];
  double jac_rt[size_residual * size_rt];
  double jac_point[size_residual * size_point];
};

TEST_F(ReprojectionError2DFixture, BrownAnalyticErrorEvaluatesOK) {
  constexpr int size = 9;

  // focal, ar, cx, cy, k1, k2, k3, p1, p2
  const double camera[size] = {0.3, 1.0, 0.001, -0.02, 0.1, -0.03, 0.001, -0.005, 0.001};
  RunTest<size>(ProjectionType::BROWN, &camera[0]);
}

TEST_F(ReprojectionError2DFixture, PerspectiveAnalyticErrorEvaluatesOK) {
  constexpr int size = 3;

  // focal, k1, k2
  const double camera[size] = {0.3, 0.1, -0.03};
  RunTest<size>(ProjectionType::PERSPECTIVE, &camera[0]);
}

TEST_F(ReprojectionError2DFixture, FisheyeAnalyticErrorEvaluatesOK) {
  constexpr int size = 3;

  // focal, k1, k2, k3
  const double camera[size] = {0.3, 0.1, -0.03};
  RunTest<size>(ProjectionType::FISHEYE, &camera[0]);
}

TEST_F(ReprojectionError2DFixture, FisheyeOpencvAnalyticErrorEvaluatesOK) {
  constexpr int size = 8;

  // focal, ar, cx, cy, k1, k2, k3, k4
  const double camera[size] = {0.3, 1.0, 0.001, -0.02, 0.1, -0.03, 0.001, -0.005};
  RunTest<size>(ProjectionType::FISHEYE_OPENCV, &camera[0]);
}

TEST_F(ReprojectionError2DFixture, DualAnalyticErrorEvaluatesOK) {
  constexpr int size = 4;

  // transtion, focal, k1, k2
  const double camera[size] = {0.5, 0.3, 0.1, -0.03};
  RunTest<size>(ProjectionType::DUAL, &camera[0]);
}

class ReprojectionError3DFixture : public ::testing::Test {
 public:
 static constexpr int size = 3;

 typedef Eigen::AutoDiffScalar<Eigen::VectorXd> AScalar;
 ReprojectionError3DFixture() { observed << 0.5, 0.5; }

 void SetupADiff() {
   const int total_size = size_point + size_rt;
   for (int i = 0; i < size_point; ++i) {
     point_adiff[i].value() = point[i];
     point_adiff[i].derivatives() = VecXd::Unit(total_size, i);
   }
   for (int i = 0; i < size_rt; ++i) {
     rt_adiff[i].value() = rt[i];
     rt_adiff[i].derivatives() = VecXd::Unit(total_size, size_point + i);
   }
  }

  void CheckJacobians() {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size_point; ++j) {
        ASSERT_NEAR(residual_adiff[i].derivatives()(j),
                    jac_point[i * size_point + j], 1e-14);
      }
    }
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size_rt; ++j) {
        ASSERT_NEAR(residual_adiff[i].derivatives()(size_point + j),
                    jac_rt[i * size_rt + j], 1e-14);
      }
    }
  }

  constexpr static int size_point = 3;
  constexpr static int size_rt = 6;

  Vec2d observed;
  double scale{0.1};
  const double point[size_point] = {1.0, 2.0, 3.0};
  const double rt[size_rt] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};

  AScalar residual_adiff[size];
  AScalar point_adiff[size_point];
  AScalar rt_adiff[size_rt];

  double residuals[size];
  double jac_rt[size * size_rt];
  double jac_point[size * size_point];
};

TEST_F(ReprojectionError3DFixture, AnalyticErrorEvaluatesOK) {
  // Autodiff-ed version will be used as reference/expected values
    SetupADiff();
    AScalar dummy_adiff;
    ReprojectionError3D autodiff(ProjectionType::SPHERICAL, observed, scale);
    autodiff(&dummy_adiff, rt_adiff, point_adiff, residual_adiff);

    // We test for analytic evaluation
    double dummy = 0.;
    double dummy_jac[] = {0., 0., 0.};
    const double* params[] = {&dummy, rt, point};
    double* jacobians[] = {&dummy_jac[0], jac_rt, jac_point};
    ReprojectionError3DAnalytic analytic(ProjectionType::SPHERICAL, observed, scale);
    analytic.Evaluate(params, residuals, &jacobians[0]);

    // Check
    CheckJacobians();
}
