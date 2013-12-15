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


#include "libmv/multiview/robust_five_point.h"
#include "libmv/multiview/fundamental.h"
#include "libmv/multiview/projection.h"
#include "libmv/multiview/nviewtriangulation.h"
#include "libmv/tools/tool.h"
#include <iostream>
#include <fstream>
#include <string>

DEFINE_double(threshold, 0.01, "Max inlier error as a portion of the image width.");
DEFINE_double(focal1, 1, "Focal length for the first camera.");
DEFINE_double(width1, 640, "Image width of the first image.");
DEFINE_double(height1, 480, "Image height of the first image.");
DEFINE_double(focal2, 1, "Focal length for the second camera.");
DEFINE_double(width2, 640, "Image width of the second image.");
DEFINE_double(height2, 480, "Image height of the second image.");

using namespace libmv;

int main(int argc, char **argv) {
  libmv::Init("", &argc, &argv);

  // Read coordinates of the matching from stdin
  std::istream *stream = &std::cin;
  std::vector<double> coordinates;
  while(stream->good()) {
    double x;
    *stream >> x;
    coordinates.push_back(x);
  }

  // Create matches matrices.
  int n_matches = coordinates.size() / 4;
  LOG(INFO) << "Num matches: " << n_matches;

  Mat x1(2, n_matches), x2(2, n_matches);
  for (int i = 0; i < n_matches; ++i) {
    x1(0, i) = coordinates[4 * i + 0];
    x1(1, i) = coordinates[4 * i + 1];
    x2(0, i) = coordinates[4 * i + 2];
    x2(1, i) = coordinates[4 * i + 3];
  }

  // Create calibration matrices.
  Mat3 K1, K2;
  K1 << FLAGS_focal1, 0, FLAGS_width1 / 2,
        0, FLAGS_focal1, FLAGS_height1 / 2,
        0, 0, 1;
  K2 << FLAGS_focal2, 0, FLAGS_width2 / 2,
        0, FLAGS_focal2, FLAGS_height2 / 2,
        0, 0, 1;

  // Compute Essential matrix.
  Mat3 E;
  vector<int> inliers;
  FivePointAlgorithmRobust(x1, x2, K1, K2,
                           FLAGS_threshold, &E, &inliers);

  LOG(INFO) << "Num inliers: " << inliers.size();

  if (inliers.size() < 5) exit(0);

  // Compute R and t.
  Mat3 R;
  Vec3 t;
  int a = inliers[inliers.size() / 2];  // Choose a random inlier.
  MotionFromEssentialAndCorrespondence(E,
                                       K1, x1.col(a),
                                       K2, x2.col(a),
                                       &R, &t);

  std::cout << R << "\n" << t << "\n";

  // Triangulate features.
  vector<Mat34> Ps(2);
  P_From_KRt(K1, Mat3::Identity(), Vec3::Zero(), &Ps[0]);
  P_From_KRt(K2, R, t, &Ps[1]);

  vector<libmv::Vec3> Xs(inliers.size());

  for (int i = 0; i < inliers.size(); ++i) {
    Mat2X x(2, 2);
    x.col(0) = x1.col(inliers[i]);
    x.col(1) = x2.col(inliers[i]);
    Vec4 X;
    NViewTriangulate(x, Ps, &X);
    Xs[i] = libmv::HomogeneousToEuclidean(X);
  }

  for (unsigned int i = 0; i < inliers.size(); ++i) {
    std::cout << inliers[i] << " "
              << Xs[i].x() << " "
              << Xs[i].y() << " "
              << Xs[i].z() << std::endl;
  }

  return 0;
}
