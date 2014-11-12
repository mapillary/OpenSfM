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

#ifndef LIBMV_MULTIVIEW_ESSENTIAL_KERNEL_H_
#define LIBMV_MULTIVIEW_ESSENTIAL_KERNEL_H_

#include <vector>
#include "libmv/multiview/fundamental_kernel.h"
#include "libmv/multiview/two_view_kernel.h"
#include "libmv/multiview/fundamental.h"
#include "libmv/numeric/numeric.h"

namespace libmv {
namespace essential {
namespace kernel {

/**
 * Eight-point algorithm for solving for the essential matrix from normalized
 * image coordinates of point correspondences.
 * See page 294 in HZ Result 11.1.
 *
 */
struct EightPointRelativePoseSolver {
  enum { MINIMUM_SAMPLES = 8 };
  static void Solve(const Mat &x1, const Mat &x2, vector<Mat3> *E);
};

// -- Generic Solver for the 5pt Essential Matrix Estimation.
// -- Need a new Class that inherit of two_view::kernel::kernel.
//     Error must be overwrite in order to compute F from E and K's.
// -- Fitting must normalize image values to camera values.
template<typename SolverArg,
  typename ErrorArg,
  typename ModelArg = Mat3>
class EssentialKernel :
  public two_view::kernel::Kernel<SolverArg, ErrorArg, ModelArg> {
 public:
  EssentialKernel(const Mat &x1, const Mat &x2,
                  const Mat3 &K1, const Mat3 &K2):
  two_view::kernel::Kernel<SolverArg, ErrorArg, ModelArg>(x1, x2),
                                                         K1_(K1), K2_(K2) {}
  void Fit(const vector<int> &samples, vector<ModelArg> *models) const {
    Mat x1 = ExtractColumns(this->x1_, samples);
    Mat x2 = ExtractColumns(this->x2_, samples);

    assert(2 == x1.rows());
    assert(SolverArg::MINIMUM_SAMPLES <= x1.cols());
    assert(x1.rows() == x2.rows());
    assert(x1.cols() == x2.cols());

    // Normalize the data (image coords to camera coords).
    Mat3 K1Inverse = K1_.inverse();
    Mat3 K2Inverse = K2_.inverse();
    Mat x1_normalized, x2_normalized;
    ApplyTransformationToPoints(x1, K1Inverse, &x1_normalized);
    ApplyTransformationToPoints(x2, K2Inverse, &x2_normalized);
    //x1_normalized = -x1_normalized;
    //x2_normalized = -x2_normalized;
    SolverArg::Solve(x1_normalized, x2_normalized, models);
  }
  double Error(int sample, const ModelArg &model) const {
    Mat3 F;
    FundamentalFromEssential(model, K1_, K2_, &F);
    return ErrorArg::Error(F, this->x1_.col(sample), this->x2_.col(sample));
  }
 protected:
  const Mat3 K1_;
  const Mat3 K2_;
};

//-- Usable solver for the 8pt Essential Matrix Estimation
typedef essential::kernel::EssentialKernel<EightPointRelativePoseSolver,
  fundamental::kernel::SampsonError, Mat3>  EightPointKernel;

}  // namespace kernel
}  // namespace essential
}  // namespace libmv

#endif  // LIBMV_MULTIVIEW_ESSENTIAL_KERNEL_H_
