
#include "libmv/multiview/robust_five_point.h"
#include "libmv/multiview/robust_panography.h"
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

  if (inliers.size() < 5) return bp::object();

  // Compute R and t.
  Mat3 R;
  Vec3 t;
  int a = inliers[inliers.size() / 2];  // Choose a random inlier.
  Mat x1inliers(2, inliers.size());
  Mat x2inliers(2, inliers.size());
  for (int i = 0; i < inliers.size(); ++i) {
    x1inliers.col(i) = x1.col(inliers[i]);
    x2inliers.col(i) = x2.col(inliers[i]);
  }
  MotionFromEssentialAndCorrespondence(E,
                                       K1, x1inliers,
                                       K2, x2inliers,
                                       &R, &t);

  // Convert results to numpy arrays.
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


bp::object Homography2pointsRobust(PyObject *x1_object,
                                   PyObject *x2_object,
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

  // Compute Essential matrix.
  Mat3 H;
  vector<int> inliers;
  HomographyFromCorrespondance2pointsRobust(x1, x2, threshold, &H, &inliers);

  LOG(INFO) << "Num inliers: " << inliers.size();

  if (inliers.size() < 2) return bp::object();

  // Convert results to numpy arrays.
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> H_row_major = H;

  bp::list retn;
  npy_intp H_shape[2] = {3, 3};
  npy_intp inliers_shape[1] = {inliers.size()};
  retn.append(bpn_array_from_data(2, H_shape, H_row_major.data()));
  retn.append(bpn_array_from_data(1, inliers_shape, &inliers[0]));

  return retn;
}

double AngleBetweenVectors(const Eigen::Vector3d &u,
                           const Eigen::Vector3d &v) {
    double c = (u.dot(v))
               / sqrt(u.dot(u) * v.dot(v));
    if (c >= 1.0) return 0.0;
    else return acos(c);
}

bp::object Triangulate(const bp::list &Ps_list,
                       const bp::list &xs_list,
                       double threshold,
                       double min_angle) {

  int n = bp::len(Ps_list);
  libmv::vector<Eigen::Matrix<double, 3, 4> > Ps;
  Eigen::Matrix<double, 2, Eigen::Dynamic> xs(2, n);
  Eigen::MatrixXd vs(3, n);
  bool angle_ok = false;
  for (int i = 0; i < n; ++i) {
    bp::object oP = Ps_list[i];
    bp::object ox = xs_list[i];

    PyArrayContiguousView<double> P_array(oP);
    PyArrayContiguousView<double> x_array(ox);

    Eigen::Map<const libmv::Mat> P(P_array.data(), 4, 3);
    Eigen::Map<const libmv::Mat> x(x_array.data(), 2, 1);

    Ps.push_back(P.transpose());
    xs.col(i) = x.col(0);

    // Check angle between rays
    if (!angle_ok) {
      Eigen::Vector3d xh;
      xh << x(0,0), x(1,0), 1;
      Eigen::Vector3d v = P.block<3,3>(0,0).transpose().inverse() * xh;
      vs.col(i) << v(0), v(1), v(2);

      for (int j = 0; j < i; ++j) {
        Eigen::Vector3d a, b;
        a << vs(0, i), vs(1, i), vs(2, i);
        b << vs(0, j), vs(1, j), vs(2, j);
        double angle = AngleBetweenVectors(a, b);
        if (angle >= min_angle) {
          angle_ok = true;
        }
      }
    }
  }

  if (!angle_ok) return bp::object();

  Eigen::Matrix<double, 4, 1> X;
  libmv::NViewTriangulateAlgebraic(xs, Ps, &X);
  X /= X(3);

  for (int i = 0; i < n; ++i) {
    Eigen::Vector3d x_reproj = Ps[i] * X;

    if (x_reproj(2) <= 0) {
      return bp::object();  
    }

    double dx = xs(0, i) - x_reproj(0) / x_reproj(2);
    double dy = xs(1, i) - x_reproj(1) / x_reproj(2);
    if (dx * dx + dy * dy > threshold * threshold) {
      return bp::object();
    }
  }

  npy_intp Xe_shape[1] = {3};
  return bpn_array_from_data(1, Xe_shape, X.data());
}

}
