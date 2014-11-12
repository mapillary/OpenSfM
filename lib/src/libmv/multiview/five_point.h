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

#ifndef LIBMV_MULTIVIEW_FIVE_POINT_H_
#define LIBMV_MULTIVIEW_FIVE_POINT_H_

#include "libmv/base/vector.h"
#include "libmv/numeric/numeric.h"

namespace libmv {

/**
 * Computes the relative pose of two calibrated cameras from 5 correspondences.
 *
 * The algorithm assumes the image points are already normalized (i.e.
 * multiplied by K^-1). The implementation uses Groebner basis, following the
 * method in [1], applying some optimization tweaks from [2].
 *
 * -# H. Stewénius, C. Engels and D. Nistér,  "Recent Developments on Direct
 *    Relative Orientation",  ISPRS 2006
 * -# D. Nistér,  "An Efficient Solution to the Five-Point Relative Pose",
 *    PAMI 2004
 *
 * \param x1  Points in the first image, one per column.
 * \param x2  Corresponding points in the second image, one per column.
 * \param Es  A list of at most 10 candidate essential matrix solutions.
 */
void FivePointsRelativePose(const Mat2X &x1, const Mat2X &x2,
                            vector<Mat3> *Es);

}  // namespace libmv

#endif  // LIBMV_MULTIVIEW_FIVE_POINT_H_
