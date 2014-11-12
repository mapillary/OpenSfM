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

#include "libmv/multiview/essential_kernel.h"
#include "libmv/multiview/projection.h"
#include "testing/testing.h"

#include "libmv/multiview/test_data_sets.h"

namespace {
using namespace libmv;

/// Check that the E matrix fit the Essential Matrix properties
/// Determinant is 0
///
#define EXPECT_ESSENTIAL_MATRIX_PROPERTIES(E, expectedPrecision) { \
  EXPECT_NEAR(0, E.determinant(), expectedPrecision); \
  Mat3 O = 2 * E * E.transpose() * E - (E * E.transpose()).trace() * E; \
  Mat3 zero3x3 = Mat3::Zero(); \
  EXPECT_MATRIX_NEAR(zero3x3, O, expectedPrecision);\
}

TEST(EightPointsRelativePose, test_data_sets) {
  // -- Setup a circular camera rig and assert that 8PT relative pose works.
  const int iNviews = 5;

  // Suppose a camera with Unit matrix as K
  NViewDataSet d = NRealisticCamerasFull(iNviews, 8,
    nViewDatasetConfigator(1, 1, 0, 0, 5, 0));

  for (int i = 0; i < iNviews; ++i) {
    vector<Mat3> Es;  // Essential,
    vector<Mat3> Rs;  // Rotation matrix.
    vector<Vec3> ts;  // Translation matrix.
    essential::kernel::EightPointRelativePoseSolver::Solve(d.x[0], d.x[1], &Es);

    // Recover rotation and translation from E.
    Rs.resize(Es.size());
    ts.resize(Es.size());
    for (int s = 0; s < Es.size(); ++s) {
      Vec2 x1Col, x2Col;
      x1Col << d.x[0].col(i)(0), d.x[0].col((i+1)%iNviews)(1);
      x2Col << d.x[1].col(i)(0), d.x[1].col((i+1)%iNviews)(1);
      CHECK(
        MotionFromEssentialAndCorrespondence(Es[s],
        d.K[0],
        x1Col,
        d.K[1],
        x2Col,
        &Rs[s],
        &ts[s]));
    }
    // -- Compute Ground Truth motion
    Mat3 R;
    Vec3 t, t0 = Vec3::Zero(), t1 = Vec3::Zero();
    RelativeCameraMotion(d.R[i], d.t[i],
                         d.R[(i+1)%iNviews],
                         d.t[(i+1)%iNviews],
                         &R, &t);

    // Assert that found relative motion is correct for almost one model.
    bool bsolution_found = false;
    for (size_t nModel = 0; nModel < Es.size(); ++nModel) {
      // Check that E holds the essential matrix constraints.
      EXPECT_ESSENTIAL_MATRIX_PROPERTIES(Es[nModel], 1e-8);

      // Check that we find the correct relative orientation.
      if (FrobeniusDistance(R, Rs[nModel]) < 1e-3
        && (t / t.norm() - ts[nModel] / ts[nModel].norm()).norm() < 1e-3 ) {
          bsolution_found = true;
      }
    }
    // -- Almost one solution must find the correct relative orientation
    CHECK(bsolution_found);
  }
}

TEST(EightPointsRelativePose_Kernel, test_data_sets) {
  typedef essential::kernel::EightPointKernel Kernel;

  int focal = 1000;
  int principal_Point = 500;
  Mat3 K;
  K << focal,     0, principal_Point,
    0, focal, principal_Point,
    0,     0, 1;

  // -- Setup a circular camera rig and assert that 8PT relative pose works.
  const int iNviews = 5;
  NViewDataSet d = NRealisticCamerasFull(iNviews, Kernel::MINIMUM_SAMPLES,
    nViewDatasetConfigator(focal, focal,
                           principal_Point, principal_Point,
                           5, 0));

  for (int i = 0; i < iNviews; ++i) {
    vector<Mat3> Es, Rs;  // Essential, Rotation matrix.
    vector<Vec3> ts;      // Translation matrix.

    // Direct value do not work.
    // As we use reference, it cannot convert Mat2X& to Mat&
    Mat x0 = d.x[0];
    Mat x1 = d.x[1];

    Kernel kernel(x0, x1, K, K);
    vector<int> samples;
    for (int k = 0; k < Kernel::MINIMUM_SAMPLES; ++k) {
      samples.push_back(k);
    }
    kernel.Fit(samples, &Es);

    // Recover rotation and translation from E.
    Rs.resize(Es.size());
    ts.resize(Es.size());
    for (int s = 0; s < Es.size(); ++s) {
      Vec2 x1Col, x2Col;
      x1Col << d.x[0].col(i)(0), d.x[0].col((i+1)%iNviews)(1);
      x2Col << d.x[1].col(i)(0), d.x[1].col((i+1)%iNviews)(1);
      CHECK(
        MotionFromEssentialAndCorrespondence(Es[s],
        d.K[0],
        x1Col,
        d.K[1],
        x2Col,
        &Rs[s],
        &ts[s]));
    }
    // -- Compute Ground Truth motion
    Mat3 R;
    Vec3 t, t0 = Vec3::Zero(), t1 = Vec3::Zero();
    RelativeCameraMotion(d.R[i], d.t[i],
                         d.R[(i+1)%iNviews],
                         d.t[(i+1)%iNviews],
                         &R, &t);

    // Assert that found relative motion is correct for almost one model.
    bool bsolution_found = false;
    for (size_t nModel = 0; nModel < Es.size(); ++nModel) {
      // Check that E holds the essential matrix constraints.
      EXPECT_ESSENTIAL_MATRIX_PROPERTIES(Es[nModel], 1e-8);

      // Check that we find the correct relative orientation.
      if (FrobeniusDistance(R, Rs[nModel]) < 1e-3
        && (t / t.norm() - ts[nModel] / ts[nModel].norm()).norm() < 1e-3 ) {
          bsolution_found = true;
      }
    }
    // -- Almost one solution must find the correct relative orientation
    CHECK(bsolution_found);
  }
}

}  // namespace
