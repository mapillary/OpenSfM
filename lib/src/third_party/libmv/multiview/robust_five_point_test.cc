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
#include <algorithm>

#include "libmv/base/vector.h"
#include "libmv/multiview/fundamental.h"
#include "libmv/multiview/robust_five_point.h"
#include "libmv/multiview/fundamental_test_utils.h"
#include "libmv/multiview/projection.h"
#include "libmv/multiview/test_data_sets.h"
#include "libmv/numeric/numeric.h"
#include "testing/testing.h"
#include "libmv/logging/logging.h"

namespace {

using namespace libmv;

TEST(RobustFivePoint, FivePointAlgorithmRobustNoOutliers) {
  TwoViewDataSet d = TwoRealisticCameras();

  Mat3 E_estimated;
  vector<int> inliers;
  FivePointAlgorithmRobust(d.x1, d.x2, d.K1, d.K2,
                           1.0, &E_estimated, &inliers);

  EXPECT_EQ(d.x1.cols(), inliers.size());

  Mat3 E_gt;
  EssentialFromRt(d.R1, d.t1, d.R2, d.t2, &E_gt);

  // Normalize.
  Mat3 E_gt_norm, E_estimated_norm;
  NormalizeFundamental(E_gt, &E_gt_norm);
  NormalizeFundamental(E_estimated, &E_estimated_norm);
  LOG(INFO) << "E_gt_norm =\n" << E_gt_norm;
  LOG(INFO) << "E_estimated_norm =\n" << E_estimated_norm;

  EXPECT_MATRIX_NEAR(E_gt_norm, E_estimated_norm, 1e-8);
}


TEST(RobustFivePoint, FivePointAlgorithmRobust) {
  TwoViewDataSet d = TwoRealisticCameras();

  d.X = 3 * Mat::Random(3, 50);
  LOG(INFO) << "X = \n" << d.X;

  Project(d.P1, d.X, &d.x1);
  Project(d.P2, d.X, &d.x2);
  LOG(INFO) << "x1 = \n" << d.x1;
  LOG(INFO) << "x2 = \n" << d.x2;

  Mat x1s, x2s;
  HorizontalStack(d.x1, 400 * Mat::Random(2, 20), &x1s);
  HorizontalStack(d.x2, 400 * Mat::Random(2, 20), &x2s);

  // Compute Essential matrix from correspondences.
  Mat3 E_estimated;
  vector<int> inliers;
  FivePointAlgorithmRobust(x1s, x2s, d.K1, d.K2,
                           1.0, &E_estimated, &inliers);

  LOG(ERROR) << "Number of inliers = " << inliers.size();
  EXPECT_LE(d.x1.cols(), inliers.size()); // Some outliers may be considered
                                          // inliers, that's fine.

  Mat3 E_gt;
  EssentialFromRt(d.R1, d.t1, d.R2, d.t2, &E_gt);

  // Normalize.
  Mat3 E_gt_norm, E_estimated_norm;
  NormalizeFundamental(E_gt, &E_gt_norm);
  NormalizeFundamental(E_estimated, &E_estimated_norm);
  LOG(INFO) << "E_gt_norm =\n" << E_gt_norm;
  LOG(INFO) << "E_estimated_norm =\n" << E_estimated_norm;

  EXPECT_MATRIX_NEAR(E_gt_norm, E_estimated_norm, 1e-8);
}



} // namespace
