// Copyright (c) 2009, 2011 libmv authors.
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

#include "libmv/logging/logging.h"
#include "libmv/multiview/fundamental_kernel.h"
#include "libmv/multiview/projection.h"
#include "libmv/multiview/test_data_sets.h"
#include "libmv/numeric/numeric.h"
#include "testing/testing.h"

using testing::Types;

namespace {

using namespace libmv;
using namespace libmv::fundamental::kernel;

// Check that sin(angle(a, b)) < tolerance.
template<typename A, typename B>
bool Colinear(const A &a, const B &b, double tolerance) {
  bool dims_match = (a.rows() == b.rows()) && (a.cols() == b.cols());
  if (!dims_match) {
    return false;
  }
  double c = CosinusBetweenMatrices(a, b);
  if (c * c < 1) {
    double s = sqrt(1 - c * c);
    return fabs(s) < tolerance;
  }
  return true;  // TODO(keir): Is this correct?
}

// Check the properties of a fundamental matrix:
//
//   1. The determinant is 0 (rank deficient)
//   2. The condition x'T*F*x = 0 is satisfied to precision.
template<typename TMat>
void ExpectFundamentalProperties(const TMat &F,
                                 const Mat &ptsA,
                                 const Mat &ptsB,
                                 double precision) {
  EXPECT_NEAR(0, F.determinant(), precision);
  assert(ptsA.cols() == ptsB.cols());
  Mat hptsA, hptsB;
  EuclideanToHomogeneous(ptsA, &hptsA);
  EuclideanToHomogeneous(ptsB, &hptsB);
  for (int i = 0; i < ptsA.cols(); ++i) {
    double residual = hptsB.col(i).dot(F * hptsA.col(i));
    EXPECT_NEAR(0.0, residual, precision);
  }
}

// Because of how type parameterized tests work, two classes are required.
template <class Kernel>
struct SevenPointTest : public testing::Test {
  void ExpectKernelProperties(const Mat &x1,
                              const Mat &x2,
                              Mat3 *F_expected = NULL) {
    Kernel kernel(x1, x2);
    vector<int> samples;
    for (int i = 0; i < x1.cols(); ++i) {
      samples.push_back(i);
    }
    vector<Mat3> Fs;
    kernel.Fit(samples, &Fs);
    bool found = false;  // Need to search for expected answer.
    EXPECT_TRUE(Fs.size() != 0);
    for (int i = 0; i < Fs.size(); ++i) {
      ExpectFundamentalProperties(Fs[i], x1, x2, 1e-8);
      if (F_expected) {
        found |= Colinear(Fs[i], *F_expected, 1e-6);
      }
    }
    if (F_expected) {
      EXPECT_TRUE(found);
    }
  }
};

typedef Types<SevenPointKernel,
              NormalizedSevenPointKernel,
              Kernel>
  SevenPointImplementations;

TYPED_TEST_CASE(SevenPointTest, SevenPointImplementations);

TYPED_TEST(SevenPointTest, EasyCase) {
  Mat x1(2, 7);
  Mat x2(2, 7);
  x1 << 0, 0, 0, 1, 1, 1, 2,
        0, 1, 2, 0, 1, 2, 0;
  x2 << 0, 0, 0, 1, 1, 1, 2,
        1, 2, 3, 1, 2, 3, 1;
  this->ExpectKernelProperties(x1, x2);
}

TYPED_TEST(SevenPointTest, RealCorrespondences) {
  Mat x1(2, 7);
  Mat x2(2, 7);

  x1 <<  723, 1091, 1691, 447,  971, 1903, 1483,
         887,  699,  811, 635,   91,  447, 1555;
  x2 << 1251, 1603, 2067, 787, 1355, 2163, 1875,
        1243,  923, 1031, 484,  363,  743, 1715;
  this->ExpectKernelProperties(x1, x2);
}

TYPED_TEST(SevenPointTest, DegeneratePointsOnCubeStillSolve) {
  // Try the 7 points of a cube and their projections, missing the last corner.
  TwoViewDataSet d = TwoRealisticCameras();
  d.X.resize(3, 7);
  d.X <<  0, 1, 0, 1, 0, 1, 0,  // X,
          0, 0, 1, 1, 0, 0, 1,  // Y,
          0, 0, 0, 0, 1, 1, 1;  // Z.
  Project(d.P1, d.X, &d.x1);
  Project(d.P2, d.X, &d.x2);
  this->ExpectKernelProperties(d.x1, d.x2, &d.F);
}

template <class Kernel>
struct EightPointTest : public SevenPointTest<Kernel> {
};

typedef Types<EightPointKernel,
              NormalizedEightPointKernel>
  EightPointImplementations;

TYPED_TEST_CASE(EightPointTest, EightPointImplementations);
TYPED_TEST(EightPointTest, EasyCase) {
  Mat x1(2, 8);
  Mat x2(2, 8);
  x1 << 0, 0, 0, 1, 1, 1, 2, 2,
        0, 1, 2, 0, 1, 2, 0, 1;
  x2 << 0, 0, 0, 1, 1, 1, 2, 2,
        1, 2, 3, 1, 2, 3, 1, 2;
  this->ExpectKernelProperties(x1, x2);
}


TYPED_TEST(EightPointTest, RealistCaseWith30Points) {
  TwoViewDataSet d = TwoRealisticCameras();
  this->ExpectKernelProperties(d.x1, d.x2, &d.F);
}

template <class Kernel>
struct FundamentalErrorTest : public testing::Test {
};

typedef Types<SampsonError,
              SymmetricEpipolarDistanceError>
  FundamentalErrorImplementations;

TYPED_TEST_CASE(FundamentalErrorTest, FundamentalErrorImplementations);
TYPED_TEST(FundamentalErrorTest, FundamentalErrorTest2) {
  Vec3 t(1, 0, 0);
  Mat3 F = CrossProductMatrix(t);  // Fundamental matrix corresponding to pure
                                   // translation.

  Vec2 x0(0, 0), y0(  0,   0);  // Good match (at infinity).
  Vec2 x1(0, 0), y1(100,   0);  // Good match (no vertical disparity).
  Vec2 x2(0, 0), y2(0.0, 0.1);  // Small error (a bit of vertical disparity).
  Vec2 x3(0, 0), y3(  0,   1);  // Bigger error.
  Vec2 x4(0, 0), y4(  0,  10);  // Biggest error.
  Vec2 x5(0, 0), y5(100,  10);  // Biggest error with horizontal disparity.

  Vec6 dists;
  dists << TypeParam::Error(F, x0, y0),
           TypeParam::Error(F, x1, y1),
           TypeParam::Error(F, x2, y2),
           TypeParam::Error(F, x3, y3),
           TypeParam::Error(F, x4, y4),
           TypeParam::Error(F, x5, y5);

  VLOG(1) << "SampsonDistance: " << dists.transpose();

  // The expected distance are two times (one per image) the distance from the
  // point to the reprojection of the best triangulated point.  For this
  // particular example this reprojection is the midpoint between the point and
  // the epipolar line.
  EXPECT_EQ(0, dists[0]);
  EXPECT_EQ(0, dists[1]);
  EXPECT_EQ(2 * Square(0.1 / 2), dists[2]);
  EXPECT_EQ(2 * Square(1.0 / 2), dists[3]);
  EXPECT_EQ(2 * Square(10. / 2), dists[4]);
  EXPECT_EQ(2 * Square(10. / 2), dists[5]);
}

}  // namespace
