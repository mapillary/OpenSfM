// Copyright (c) 2009 libmv authors.
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

#ifndef LIBMV_MULTIVIEW_FIVE_POINT_KERNEL_H_
#define LIBMV_MULTIVIEW_FIVE_POINT_KERNEL_H_

#include "libmv/multiview/fundamental_kernel.h"
#include "libmv/multiview/essential_kernel.h"
#include "libmv/multiview/five_point.h"
#include "libmv/numeric/numeric.h"

namespace libmv {
namespace essential {
namespace kernel {

struct FivePointSolver {
  enum { MINIMUM_SAMPLES = 5 };
  static void Solve(const Mat &x1, const Mat &x2, vector<Mat3> *Es) {
    FivePointsRelativePose(x1, x2, Es);
  }
};

// Kernel for five point essential matrix estimation.
typedef essential::kernel::EssentialKernel<FivePointSolver,
  fundamental::kernel::SampsonError, Mat3>  FivePointKernel;

}  // namespace kernel
}  // namespace essential
}  // namespace libmv

#endif  // LIBMV_MULTIVIEW_FIVE_POINT_KERNEL_H_

