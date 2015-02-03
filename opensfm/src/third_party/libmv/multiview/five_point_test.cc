// Copyright (c) 2007, 2008 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#include <iostream>

#include "testing/testing.h"
#include "libmv/logging/logging.h"
#include "libmv/numeric/numeric.h"
#include "libmv/multiview/fundamental.h"
#include "libmv/multiview/projection.h"
#include "libmv/multiview/five_point.h"
#include "libmv/multiview/five_point_internal.h"
#include "libmv/multiview/test_data_sets.h"

namespace libmv {
namespace {

struct TestData {
  Mat3X X;
  Mat3 R;
  Vec3 t;
  Mat3 E;
  Mat34 P1, P2;
  Mat2X x1, x2;
};

TestData SomeTestData() {
  TestData d;
  d.X = Mat3X::Random(3, 5);
  d.X.row(0).array() -= .5;
  d.X.row(1).array() -= .5;
  d.X.row(2).array() += 3;
  d.R = RotationAroundZ(0.3) * RotationAroundX(0.1) * RotationAroundY(0.2);
  d.t = Vec3::Random();

  EssentialFromRt(Mat3::Identity(), Vec3::Zero(), d.R, d.t, &d.E);

  P_From_KRt(Mat3::Identity(), Mat3::Identity(), Vec3::Zero(), &d.P1);
  P_From_KRt(Mat3::Identity(), d.R, d.t, &d.P2);
  Project(d.P1, d.X, &d.x1);
  Project(d.P2, d.X, &d.x2);
  return d;
}


TEST(FivePointsNullspaceBasis, SatisfyEpipolarConstraint) {
  TestData d = SomeTestData();

  Mat E_basis = FivePointsNullspaceBasis(d.x1, d.x2);

  for (int s = 0; s < 4; ++s) {
    Mat3 E;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        E(i, j) = E_basis(3 * i + j, s);
      }
    }
    for (int i = 0; i < d.x1.cols(); ++i) {
      Vec3 x1(d.x1(0, i), d.x1(1, i), 1);
      Vec3 x2(d.x2(0, i), d.x2(1, i), 1);
      EXPECT_NEAR(0, x2.dot(E * x1), 1e-6);
    }
  }
}

double EvalPolynomial(Vec p, double x, double y, double z) {
  return p(coef_xxx) * x * x * x
       + p(coef_xxy) * x * x * y
       + p(coef_xxz) * x * x * z
       + p(coef_xyy) * x * y * y
       + p(coef_xyz) * x * y * z
       + p(coef_xzz) * x * z * z
       + p(coef_yyy) * y * y * y
       + p(coef_yyz) * y * y * z
       + p(coef_yzz) * y * z * z
       + p(coef_zzz) * z * z * z
       + p(coef_xx)  * x * x
       + p(coef_xy)  * x * y
       + p(coef_xz)  * x * z
       + p(coef_yy)  * y * y
       + p(coef_yz)  * y * z
       + p(coef_zz)  * z * z
       + p(coef_x)   * x
       + p(coef_y)   * y
       + p(coef_z)   * z
       + p(coef_1)   * 1;
}

TEST(o1, Evaluation) {
  Vec p1 = Vec::Zero(20), p2 = Vec::Zero(20);
  p1(coef_x) = double(rand()) / RAND_MAX;
  p1(coef_y) = double(rand()) / RAND_MAX;
  p1(coef_z) = double(rand()) / RAND_MAX;
  p1(coef_1) = double(rand()) / RAND_MAX;
  p2(coef_x) = double(rand()) / RAND_MAX;
  p2(coef_y) = double(rand()) / RAND_MAX;
  p2(coef_z) = double(rand()) / RAND_MAX;
  p2(coef_1) = double(rand()) / RAND_MAX;

  Vec p3 = o1(p1, p2);

  for (double z = -5; z < 5; ++z) {
    for (double y = -5; y < 5; ++y) {
      for (double x = -5; x < 5; ++x) {
        EXPECT_NEAR(EvalPolynomial(p3, x, y, z),
                    EvalPolynomial(p1, x, y, z) * EvalPolynomial(p2, x, y, z),
                    1e-8);
      }
    }
  }
}

TEST(o2, Evaluation) {
  Vec p1 = Vec::Zero(20), p2 = Vec::Zero(20);
  p1(coef_xx) = double(rand()) / RAND_MAX;
  p1(coef_xy) = double(rand()) / RAND_MAX;
  p1(coef_xz) = double(rand()) / RAND_MAX;
  p1(coef_yy) = double(rand()) / RAND_MAX;
  p1(coef_yz) = double(rand()) / RAND_MAX;
  p1(coef_zz) = double(rand()) / RAND_MAX;
  p1(coef_x)  = double(rand()) / RAND_MAX;
  p1(coef_y)  = double(rand()) / RAND_MAX;
  p1(coef_z)  = double(rand()) / RAND_MAX;
  p1(coef_1)  = double(rand()) / RAND_MAX;
  p2(coef_x)  = double(rand()) / RAND_MAX;
  p2(coef_y)  = double(rand()) / RAND_MAX;
  p2(coef_z)  = double(rand()) / RAND_MAX;
  p2(coef_1)  = double(rand()) / RAND_MAX;

  Vec p3 = o2(p1, p2);

  for (double z = -5; z < 5; ++z) {
    for (double y = -5; y < 5; ++y) {
      for (double x = -5; x < 5; ++x) {
        EXPECT_NEAR(EvalPolynomial(p3, x, y, z),
                    EvalPolynomial(p1, x, y, z) * EvalPolynomial(p2, x, y, z),
                    1e-8);
      }
    }
  }
}

TEST(FivePointsGaussJordan, RandomMatrix) {
  Mat M = Mat::Random(10, 20);
  FivePointsGaussJordan(&M);
  Mat I = Mat::Identity(10, 10);
  Mat M1 = M.block<10, 10>(0, 0);
  EXPECT_MATRIX_NEAR(I, M1, 1e-8);
}

TEST(FivePointsRelativePose, Random) {
  TestData d = SomeTestData();

  vector<Mat3> Es;
  vector<Mat3> Rs;
  vector<Vec3> ts;
  FivePointsRelativePose(d.x1, d.x2, &Es);

  // Recover rotation and translation from E
  Rs.resize(Es.size());
  ts.resize(Es.size());
  for (int s = 0; s < Es.size(); ++s) {
    MotionFromEssentialAndCorrespondence(Es[s],
                                         Mat3::Identity(),
                                         d.x1.col(0),
                                         Mat3::Identity(),
                                         d.x2.col(0),
                                         &Rs[s],
                                         &ts[s]);
  }

  bool solution_found = false;
  for (int i = 0; i < Rs.size(); ++i) {
    // Check that E holds the essential matrix constraints.
    Mat3 E = Es[i];
    EXPECT_NEAR(0, E.determinant(), 1e-8);
    Mat3 O = 2 * E * E.transpose() * E - (E * E.transpose()).trace() * E;
    EXPECT_MATRIX_NEAR(Mat3::Zero(), O, 1e-8);

    // Check that we find the correct relative orientation.
    if (FrobeniusDistance(d.R, Rs[i]) < 1e-3
        && (d.t / d.t.norm() - ts[i] / ts[i].norm()).norm() < 1e-3) {
      solution_found = true;
      break;
    }
  }
  EXPECT_TRUE(solution_found);
}

TEST(FivePointsRelativePose, test_data_sets) {
  // Setup a circular camera rig and assert that 5PT relative pose works.
  const int num_views = 5;

  // Suppose a camera with Unit matrix as K
  NViewDataSet d = NRealisticCamerasFull(num_views, 5,
    nViewDatasetConfigator(1, 1, 0, 0, 5, 0));

  for (int i = 0; i < num_views; ++i) {
    vector<Mat3> Es, Rs;  // Essential, Rotation matrix.
    vector<Vec3> ts;      // Translation matrix.
    FivePointsRelativePose(d.x[0], d.x[1], &Es);

    // Recover rotation and translation from E.
    Rs.resize(Es.size());
    ts.resize(Es.size());
    for (int s = 0; s < Es.size(); ++s) {
      Vec2 x1Col, x2Col;
      x1Col << d.x[0].col(i)(0), d.x[0].col((i+1) % num_views)(1);
      x2Col << d.x[1].col(i)(0), d.x[1].col((i+1) % num_views)(1);
      MotionFromEssentialAndCorrespondence(Es[s],
                                           d.K[0],
                                           x1Col,
                                           d.K[1],
                                           x2Col,
                                           &Rs[s],
                                           &ts[s]);
    }
    // Compute the ground truth motion.
    Mat3 R;
    Vec3 t, t0 = Vec3::Zero(), t1 = Vec3::Zero();
    RelativeCameraMotion(d.R[i], d.t[i],
                         d.R[(i + 1) % num_views],
                         d.t[(i + 1) % num_views],
                         &R, &t);

    // Assert that the found relative motion is correct for at least one model.
    bool bsolution_found = false;
    for (size_t j = 0; j < Es.size(); ++j) {
      // Check that E holds the essential matrix constraints.
      Mat3 E = Es[j];
      EXPECT_NEAR(0, E.determinant(), 1e-8);
      Mat3 O = 2 * E * E.transpose() * E - (E * E.transpose()).trace() * E;
      EXPECT_MATRIX_NEAR(Mat3::Zero(), O, 1e-8);

      // Check that we find the correct relative orientation.
      if (FrobeniusDistance(R, Rs[j]) < 1e-3
        && (t / t.norm() - ts[j] / ts[j].norm()).norm() < 1e-3 ) {
          bsolution_found = true;
      }
    }
    // At least one solution must find the correct relative orientation.
    CHECK(bsolution_found);
  }
}

}  // namespace
}  // namespace libmv
