
#include "libmv/multiview/robust_five_point.h"
#include "libmv/multiview/robust_panography.h"
#include "libmv/multiview/fundamental.h"
#include "libmv/multiview/projection.h"
#include "libmv/multiview/nviewtriangulation.h"
#include "libmv/base/vector.h"
#include "libmv/multiview/random_sample.h"
#include "libmv/multiview/robust_estimation.h"
#include "libmv/numeric/numeric.h"
#include "libmv/tools/tool.h"
#include <iostream>
#include <fstream>
#include <string>
#include "types.h"
#include "bundle_two_views.h"

namespace csfm {

using namespace libmv;


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
                           threshold, &E, &inliers, 1e-4);

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

  double r[3];
  ceres::RotationMatrixToAngleAxis(&R(0,0), r);
  double covariance[6 * 6];
  if (true) {
    TwoViewBundleAdjuster ba;
    ba.InitParams(r[0], r[1], r[2], t[0], t[1], t[2], focal1, focal2);
    for (int i = 0; i < x1.cols(); ++i) {
      ba.AddObservation(x1(0, i), x1(1, i), x2(0, i), x2(1, i));
    }
    ba.SetReprojectionErrorSD(threshold);
    ba.SetLossFunction("CauchyLoss", 1);
    ba.SetComputeCovariance(true);
    ba.Run();
    TVBAParams p = ba.GetParams();
    r[0] = p.GetRX();
    r[1] = p.GetRY();
    r[2] = p.GetRZ();
    t[0] = p.GetTX();
    t[1] = p.GetTY();
    t[2] = p.GetTZ();
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        covariance[6 * i + j] = p.GetCovariance(i, j);
      }
    }
  }

  // Convert results to numpy arrays.
  bp::list retn;
  npy_intp r_shape[1] = {3};
  npy_intp t_shape[1] = {3};
  npy_intp covariance_shape[2] = {6, 6};
  npy_intp inliers_shape[1] = {inliers.size()};
  retn.append(bpn_array_from_data(1, r_shape, r));
  retn.append(bpn_array_from_data(1, t_shape, t.data()));
  retn.append(bpn_array_from_data(2, covariance_shape, covariance));
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

enum {
  TRIANGULATION_OK = 0,
  TRIANGULATION_SMALL_ANGLE,
  TRIANGULATION_BEHIND_CAMERA,
  TRIANGULATION_BAD_REPROJECTION
};

bp::object TriangulateReturn(int error, bp::object value) {
    bp::list retn;
    retn.append(int(error));
    retn.append(value);
    return retn;
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

  if (!angle_ok) {
    return TriangulateReturn(TRIANGULATION_SMALL_ANGLE, bp::object());
  }

  Eigen::Matrix<double, 4, 1> X;
  libmv::NViewTriangulateAlgebraic(xs, Ps, &X);
  X /= X(3);

  for (int i = 0; i < n; ++i) {
    Eigen::Vector3d x_reproj = Ps[i] * X;

    if (x_reproj(2) <= 0) {
      return TriangulateReturn(TRIANGULATION_BEHIND_CAMERA, bp::object());
    }

    double dx = xs(0, i) - x_reproj(0) / x_reproj(2);
    double dy = xs(1, i) - x_reproj(1) / x_reproj(2);
    if (dx * dx + dy * dy > threshold * threshold) {
     return TriangulateReturn(TRIANGULATION_BAD_REPROJECTION, bp::object());
    }
  }

  npy_intp Xe_shape[1] = {3};
  return TriangulateReturn(TRIANGULATION_OK,
      bpn_array_from_data(1, Xe_shape, X.data()));
}


struct PoseKnownRotationKernel {

  PoseKnownRotationKernel(const Mat &v, const Mat &X) : v_(v), X_(X) {}

  typedef Vec3 Model;  // camera center

  enum { MINIMUM_SAMPLES = 2 };

  int NumSamples() const {
    return v_.cols();
  }

  void Fit(const vector<int> &samples, vector<Vec3> *cs) const {
    assert(samples.size() >= (uint)MINIMUM_SAMPLES);
    Mat3X sampled_v = ExtractColumns(v_, samples);
    Mat3X sampled_X = ExtractColumns(X_, samples);
    Eigen::Matrix<double, 6, 5>  A;
    A << 1.0, 0.0, 0.0, sampled_v(0, 0), 0.0,
         0.0, 1.0, 0.0, sampled_v(1, 0), 0.0,
         0.0, 0.0, 1.0, sampled_v(2, 0), 0.0,
         1.0, 0.0, 0.0, 0.0, sampled_v(0, 1),
         0.0, 1.0, 0.0, 0.0, sampled_v(1, 1),
         0.0, 0.0, 1.0, 0.0, sampled_v(2, 1);
    Eigen::Matrix<double, 6, 1>  sampled_X_vec;
    sampled_X_vec << sampled_X(0), sampled_X(1), sampled_X(2),
                    sampled_X(3), sampled_X(4), sampled_X(5);
    Mat B(A.transpose() * A);
    Vec b(A.transpose() * sampled_X_vec);
    Vec5 x = B.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);
    Vec3 c;
    c << x(0), x(1), x(2);
    cs->push_back(c);
  }

  double Error(int sample, const Vec3 &c) const {
    Vec3 sampled_v;
    Vec3 sampled_X;
    MatrixColumn(v_, sample, &sampled_v);
    MatrixColumn(X_, sample, &sampled_X);
    double nv = NormalizeL2(&sampled_v);
    Vec3 d = sampled_X - c;
    double nd = NormalizeL2(&d);
    double error = DistanceL2(sampled_v, d);
    return error;
  }

  const Mat &v_;
  const Mat &X_;
};


double PoseKnownRotationRobustSolver(const Mat &v,
                                const Mat &X,
                                double threshold,
                                Vec3 *c,
                                vector<int> *inliers) {
  double best_score = HUGE_VAL;
  double alarm_rate =  1.0*1e-2;
  PoseKnownRotationKernel kernel(v, X);
  MLEScorer<PoseKnownRotationKernel> scorer(threshold);
  *c = Estimate(kernel, scorer, inliers,
                &best_score, alarm_rate);
  if (best_score == HUGE_VAL)
    return HUGE_VAL;
  else
    return best_score;
}

bp::object PoseKnownRotationRobust(PyObject *v_object,
                                   PyObject *X_object,
                                   double threshold
                                   ) {
  using namespace libmv;

  PyArrayContiguousView<double> v_array((PyArrayObject *)v_object);
  PyArrayContiguousView<double> X_array((PyArrayObject *)X_object);

  assert(v_array.shape(0) == x2_array.shape(0));
  assert(v_array.shape(1) == 3);
  assert(X_array.shape(1) == 3);

  int n_point = v_array.shape(0);

  Eigen::Map<const libmv::Mat> v(v_array.data(), 3, n_point);
  Eigen::Map<const libmv::Mat> X(X_array.data(), 3, n_point);

  // Compute camera center
  Vec3 c;
  vector<int> inliers;
  double error = PoseKnownRotationRobustSolver(v, X, threshold, &c, &inliers);

  if (inliers.size() < 2) return bp::object();

  // Convert results to numpy arrays.
  bp::list retn;
  npy_intp c_shape[2] = {3, 1};
  npy_intp inliers_shape[1] = {inliers.size()};
  retn.append(bpn_array_from_data(2, c_shape, c.data()));
  retn.append(bpn_array_from_data(1, inliers_shape, &inliers[0]));
  return retn;
}

}

