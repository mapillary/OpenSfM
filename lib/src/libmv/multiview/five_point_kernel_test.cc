// Copyright (c) 2010 libmv authors.
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

#include "libmv/multiview/five_point_kernel.h"
#include "libmv/multiview/fundamental.h"
#include "libmv/multiview/test_data_sets.h"
#include "testing/testing.h"

namespace libmv {
namespace {

// Check that the E matrix has the essential matrix properties; in other words,
// the determinant is zero, and the other two singular values are the same.
#define EXPECT_ESSENTIAL_MATRIX_PROPERTIES(E, expectedPrecision) { \
  EXPECT_NEAR(0, E.determinant(), expectedPrecision); \
  Mat3 O = 2 * E * E.transpose() * E - (E * E.transpose()).trace() * E; \
  Mat3 zero3x3 = Mat3::Zero(); \
  EXPECT_MATRIX_NEAR(zero3x3, O, expectedPrecision); \
}

TEST(FivePointsRelativePoseKernel, CircularCameraRig) {
  typedef essential::kernel::FivePointKernel Kernel;

  // Create a realistic camera configuration.
  int focal = 1000;
  int principal_point = 500;
  Mat3 K;
  K << focal,     0, principal_point,
           0, focal, principal_point,
           0,     0, 1;

  // Create a circular camera rig and assert that five point relative pose works.
  const int num_views = 5;
  NViewDataSet d = NRealisticCamerasFull(
      num_views,
      Kernel::MINIMUM_SAMPLES,
      nViewDatasetConfigator(focal,
                             focal,
                             principal_point,
                             principal_point,
                             5,
                             0));

  for (int i = 0; i < num_views; ++i)  {
    vector<Mat3> Es, Rs;  // Essential and rotation matrices.
    vector<Vec3> ts;      // Translation matrices.

    // Use a copy, since d.x[n] is a Mat2x&.
    Mat x0 = d.x[0];
    Mat x1 = d.x[1];

    Kernel kernel(x0, x1, K, K);
    vector<int> samples;
    for (int k = 0; k < Kernel::MINIMUM_SAMPLES; ++k) {
      samples.push_back(k);
    }
    kernel.Fit(samples, &Es);

    // Recover rotation and translation frm E.
    Rs.resize(Es.size());
    ts.resize(Es.size());
    for (int s = 0; s < Es.size(); ++s) {
      Vec2 x1Col, x2Col;
      x1Col << d.x[0].col(i)(0), d.x[0].col((i + 1) % num_views)(1);
      x2Col << d.x[1].col(i)(0), d.x[1].col((i + 1) % num_views)(1);
      CHECK(MotionFromEssentialAndCorrespondence(Es[s],
            d.K[0], x1Col,
            d.K[1], x2Col,
            &Rs[s],
            &ts[s]));
    }

    // Compute the ground truth motion.
    Mat3 R;
    Vec3 t, t0 = Vec3::Zero(), t1 = Vec3::Zero();
    RelativeCameraMotion(d.R[i],
                         d.t[i],
                         d.R[(i + 1) % num_views],
                         d.t[(i + 1) % num_views],
                         &R, &t);

    // Assert that found relative motion is correct for at least one model.
    bool solution_found = false;
    for (size_t j = 0; j < Es.size(); ++j) {
      // Check that E holds the essential matrix constraints.
      EXPECT_ESSENTIAL_MATRIX_PROPERTIES(Es[j], 1e-8);

      // Check residuals
      for (int k = 0; k < x1.cols(); ++k) {
        EXPECT_NEAR(0.0, kernel.Error(k, Es[j]), 1e-8);
      }

      // Check that we find the correct relative orientation.
      if (FrobeniusDistance(R, Rs[j]) < 1e-3
        && (t / t.norm() - ts[j] / ts[j].norm()).norm() < 1e-3 ) {
          solution_found = true;
      }
    }
    CHECK(solution_found);
  }
}

}  // namespace
}  // namespace libmv
