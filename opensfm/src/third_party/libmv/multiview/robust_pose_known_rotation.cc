#include "libmv/base/vector.h"
#include "libmv/multiview/random_sample.h"
#include "libmv/multiview/robust_estimation.h"
#include "libmv/numeric/numeric.h"
#include <iostream>
#include <fstream>
#include <string>

namespace libmv {

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
    for (int i = 0; i < 6; ++i){
      for (int j =0; j < 5; ++j){
        A(i, j) = 0.0;
      }
    }
    A(0, 0) = 1.0;
    A(1, 1) = 1.0;
    A(2, 2) = 1.0;
    A(3, 0) = 1.0;
    A(4, 1) = 1.0;
    A(5, 2) = 1.0;
    A(0, 3) = sampled_v(0, 0);
    A(1, 3) = sampled_v(1, 0);
    A(2, 3) = sampled_v(2, 0);
    A(3, 4) = sampled_v(0, 1);
    A(4, 4) = sampled_v(1, 1);
    A(5, 4) = sampled_v(2, 1);

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

double PoseKnownRotationRobust(const Mat &v,
                                const Mat &X,
                                double threshold,
                                Vec3 *c,
                                vector<int> *inliers) {
  double best_score = HUGE_VAL;
  double alarm_rate = 1e-2;
  PoseKnownRotationKernel kernel(v, X);
  MLEScorer<PoseKnownRotationKernel> scorer(threshold);
  *c = Estimate(kernel, scorer, inliers,
                &best_score, alarm_rate);
  if (best_score == HUGE_VAL)
    return HUGE_VAL;
  else
    return best_score;
}

} // libmv