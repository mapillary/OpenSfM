
#include "libmv/multiview/robust_five_point.h"
#include "libmv/multiview/fundamental.h"
#include "libmv/multiview/projection.h"
#include "libmv/multiview/nviewtriangulation.h"
#include "libmv/tools/tool.h"
#include <iostream>
#include <fstream>
#include <string>
#include "types.h"

namespace csfm {

bp::object TwoViewReconstruction(PyObject *x1_object,
                                 PyObject *x2_object,
                                 double focal1,
                                 double focal2,
                                 double threshold) {
  using namespace libmv;

  PyArrayContiguousView<double> x1_array((PyArrayObject *)x1_object);
  PyArrayContiguousView<double> x2_array((PyArrayObject *)x2_object);

  assert(x1_array.shape(0) == x2_array.shape(0));
  assert(x1_array.shape(1) == 2);
  assert(x2_array.shape(1) == 2);

  // Create matches matrices.
  int n_matches = x1_array.shape(0);
  LOG(INFO) << "Num matches: " << n_matches;

  if (n_matches < 5) return bp::object();

  Eigen::Map<const libmv::Mat> x1(x1_array.data(), 2, n_matches);
  Eigen::Map<const libmv::Mat> x2(x2_array.data(), 2, n_matches);

  // Create calibration matrices.
  Mat3 K1, K2;
  K1 << focal1, 0, 0,
        0, focal1, 0,
        0, 0, 1;
  K2 << focal2, 0, 0,
        0, focal2, 0,
        0, 0, 1;

  // Compute Essential matrix.
  Mat3 E;
  vector<int> inliers;
  FivePointAlgorithmRobust(x1, x2, K1, K2,
                           threshold, &E, &inliers);

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

  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_row_major = R;

  bp::list retn;
  npy_intp R_shape[2] = {3, 3};
  npy_intp t_shape[1] = {3};
  npy_intp inliers_shape[1] = {inliers.size()};
  retn.append(bpn_array_from_data(2, R_shape, R_row_major.data()));
  retn.append(bpn_array_from_data(1, t_shape, t.data()));
  retn.append(bpn_array_from_data(1, inliers_shape, &inliers[0]));

  return retn;
}

}
