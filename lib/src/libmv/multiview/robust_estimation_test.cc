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

#include "libmv/base/vector.h"
#include "libmv/multiview/random_sample.h"
#include "libmv/multiview/robust_estimation.h"
#include "libmv/numeric/numeric.h"
#include "testing/testing.h"

namespace {

using namespace libmv;

// TODO(keir): This belongs in random_sample_test.cc.
TEST(UniformSampleTest, NoRepetions) {
  vector<int> samples;
  for (int total = 1; total < 500; total *= 2) {
    for (int num_samples = 1; num_samples <= total; num_samples *= 2) {
      UniformSample(num_samples, total, &samples);
      std::vector<bool> in_set(total, false);
      for (int i = 0; i < num_samples; ++i) {
        EXPECT_FALSE(in_set[samples[i]]);
        in_set[samples[i]] = true;
      }
    }
  }
}

struct LineKernel {
  LineKernel(const Mat2X &xs) : xs_(xs) {}

  typedef Vec2 Model;  // a, b;

  enum { MINIMUM_SAMPLES = 2 };

  int NumSamples() const {
    return xs_.cols();
  }

  void Fit(const vector<int> &samples, vector<Vec2> *lines) const {
    assert(samples.size() >= (uint)MINIMUM_SAMPLES);
    // Standard least squares solution.
    Mat2X sampled_xs = ExtractColumns(xs_, samples);
    MatX2 X(sampled_xs.cols(), 2);
    X.col(0).setOnes();
    X.col(1) = sampled_xs.row(0).transpose();
    Mat A(X.transpose() * X);
    Vec b(X.transpose() * sampled_xs.row(1).transpose());
    Vec2 ba = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);
    lines->push_back(ba);
  }

  double Error(int sample, const Vec2 &ba) const {
    double b = ba[0];
    double a = ba[1];
    double x = xs_(0, sample);
    double y = xs_(1, sample);
    double e = y - (a*x + b);
    return e*e;
  }

  const Mat2X &xs_;
};

// Since the line fitter isn't so simple, test it in isolation.
TEST(LineFitter, ItWorks) {
  Mat2X xy(2, 5);
  // y = 2x + 1
  xy << 1, 2, 3, 4,  5,
        3, 5, 7, 9, 11;
  vector<Vec2> models;
  LineKernel kernel(xy);
  vector<int> samples;
  for (int i = 0; i < xy.cols(); ++i) {
    samples.push_back(i);
  }
  kernel.Fit(samples, &models);
  ASSERT_EQ(1, models.size());
  EXPECT_NEAR(2.0, models[0][1], 1e-9);
  EXPECT_NEAR(1.0, models[0][0], 1e-9);
}

TEST(RobustLineFitter, OutlierFree) {
  Mat2X xy(2, 5);
  // y = 2x + 1
  xy << 1, 2, 3, 4,  5,
        3, 5, 7, 9, 11;

  LineKernel kernel(xy);
  vector<int> inliers;
  Vec2 ba = Estimate(kernel, MLEScorer<LineKernel>(4), &inliers);
  EXPECT_NEAR(2.0, ba[1], 1e-9);
  EXPECT_NEAR(1.0, ba[0], 1e-9);
  ASSERT_EQ(5, inliers.size());
}

TEST(RobustLineFitter, OutlierFree_DoNotGetBackInliers) {
  Mat2X xy(2, 5);
  // y = 2x + 1
  xy << 1, 2, 3, 4,  5,
        3, 5, 7, 9, 11;

  LineKernel kernel(xy);
  Vec2 ba = Estimate(kernel, MLEScorer<LineKernel>(4));
  EXPECT_NEAR(2.0, ba[1], 1e-9);
  EXPECT_NEAR(1.0, ba[0], 1e-9);
}

TEST(RobustLineFitter, OneOutlier) {
  Mat2X xy(2, 6);
  // y = 2x + 1 with an outlier
  xy << 1, 2, 3, 4,  5, /* outlier! */  100,
        3, 5, 7, 9, 11, /* outlier! */ -123;

  LineKernel kernel(xy);
  vector<int> inliers;
  Vec2 ba = Estimate(kernel, MLEScorer<LineKernel>(4), &inliers);
  EXPECT_NEAR(2.0, ba[1], 1e-9);
  EXPECT_NEAR(1.0, ba[0], 1e-9);
  ASSERT_EQ(5, inliers.size());
}

// Test if the robust estimator do not return inlier if too few point
// was given for an estimation.
TEST(RobustLineFitter, TooFewPoints) {
  Mat2X xy(2, 1);
  // y = 2x + 1
  xy << 1,
        3;
  LineKernel kernel(xy);
  vector<int> inliers;
  Vec2 ba = Estimate(kernel, MLEScorer<LineKernel>(4), &inliers);
  (void) ba;
  ASSERT_EQ(0, inliers.size());
}

}  // namespace
