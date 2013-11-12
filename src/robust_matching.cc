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


#include "libmv/multiview/robust_fundamental.h"
#include "libmv/tools/tool.h"
#include <iostream>
#include <fstream>
#include <string>

DEFINE_double(threshold, 0.01, "Max inlier error as a portion of the image width.");

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

  Mat3 F;
  vector<int> inliers;
  FundamentalFromCorrespondences7PointRobust(x1, x2, FLAGS_threshold, &F, &inliers);

  LOG(INFO) << "Num inliers: " << inliers.size();

  for (unsigned int i = 0; i < inliers.size(); ++i) {
    std::cout << inliers[i] << std::endl;
  }

  return 0;
}
