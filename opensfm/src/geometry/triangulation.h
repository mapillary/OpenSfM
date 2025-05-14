#pragma once

#include <foundation/types.h>

#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <Eigen/StdVector>

namespace geometry {

std::pair<bool, Vec3d> TriangulateBearingsDLT(const std::vector<Mat34d> &Rts,
                                              const MatX3d &bearings,
                                              double threshold,
                                              double min_angle,
                                              double min_depth);

Vec4d TriangulateBearingsDLTSolve(const MatX3d &bs,
                                  const std::vector<Mat34d> &Rts);

std::pair<bool, Vec3d> TriangulateBearingsMidpoint(
    const MatX3d &centers, const MatX3d &bearings,
    const std::vector<double> &threshold_list, double min_angle,
    double min_depth);

template <class T>
Eigen::Matrix<T, 3, 1> TriangulateBearingsMidpointSolve(
    const Eigen::Matrix<T, Eigen::Dynamic, 3> &centers,
    const Eigen::Matrix<T, Eigen::Dynamic, 3> &bearings);

std::vector<std::pair<bool, Vec3d>> TriangulateTwoBearingsMidpointMany(
    const MatX3d &bearings1, const MatX3d &bearings2, const Mat3d &rotation,
    const Vec3d &translation);

template <class T>
std::pair<bool, Eigen::Matrix<T, 3, 1>> TriangulateTwoBearingsMidpointSolve(
    const Eigen::Matrix<T, 2, 3> &centers,
    const Eigen::Matrix<T, 2, 3> &bearings);

MatXd EpipolarAngleTwoBearingsMany(const MatX3d &bearings1,
                                   const MatX3d &bearings2,
                                   const Mat3d &rotation,
                                   const Vec3d &translation);

Vec3d PointRefinement(const MatX3d &centers, const MatX3d &bearings,
                      const Vec3d &point, int iterations);

// Template implementations

// Point minimizing the squared distance to all rays
// Closed for solution from
//   Srikumar Ramalingam, Suresh K. Lodha and Peter Sturm
//   "A generic structure-from-motion framework"
//   CVIU 2006
template <class T>
Eigen::Matrix<T, 3, 1> TriangulateBearingsMidpointSolve(
    const Eigen::Matrix<T, Eigen::Dynamic, 3> &centers,
    const Eigen::Matrix<T, Eigen::Dynamic, 3> &bearings) {
  int nviews = bearings.rows();
  assert(nviews == centers.rows());
  assert(nviews >= 2);

  Eigen::Matrix<T, 3, 3> BBt;
  Eigen::Matrix<T, 3, 1> BBtA, A;
  BBt.setZero();
  BBtA.setZero();
  A.setZero();
  for (int i = 0; i < nviews; ++i) {
    BBt += bearings.row(i).transpose() * bearings.row(i);
    BBtA += bearings.row(i).transpose() * bearings.row(i) *
            centers.row(i).transpose();
    A += centers.row(i);
  }
  Eigen::Matrix<T, 3, 3> Cinv =
      (T(nviews) * Eigen::Matrix<T, 3, 3>::Identity() - BBt).inverse();

  return (Eigen::Matrix<T, 3, 3>::Identity() + BBt * Cinv) * A / T(nviews) -
         Cinv * BBtA;
}

template <class T>
std::pair<bool, Eigen::Matrix<T, 3, 1>> TriangulateTwoBearingsMidpointSolve(
    const Eigen::Matrix<T, 2, 3> &centers,
    const Eigen::Matrix<T, 2, 3> &bearings) {
  const auto translation = centers.row(1) - centers.row(0);
  Eigen::Matrix<T, 2, 1> b;
  b[0] = translation.dot(bearings.row(0));
  b[1] = translation.dot(bearings.row(1));
  Eigen::Matrix<T, 2, 2> A;
  A(0, 0) = bearings.row(0).dot(bearings.row(0));
  A(1, 0) = bearings.row(0).dot(bearings.row(1));
  A(0, 1) = -A(1, 0);
  A(1, 1) = -bearings.row(1).dot(bearings.row(1));

  const T eps = T(1e-10);
  const T det = A.determinant();

  if (-eps < det && det < eps) {
    return std::make_pair(false, Eigen::Matrix<T, 3, 1>());
  }
  const auto lambdas = A.inverse() * b;
  const auto x1 = centers.row(0) + lambdas[0] * bearings.row(0);
  const auto x2 = centers.row(1) + lambdas[1] * bearings.row(1);
  return std::make_pair(true, (x1 + x2) / T(2.0));
}

}  // namespace geometry
