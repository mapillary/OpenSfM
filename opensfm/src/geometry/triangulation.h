#pragma once

#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <Eigen/StdVector>
#include <fstream>
#include <iostream>
#include <string>

double AngleBetweenVectors(const Eigen::Vector3d &u, const Eigen::Vector3d &v);

Eigen::Vector4d TriangulateBearingsDLTSolve(
    const Eigen::Matrix<double, -1, 3> &bs,
    const std::vector<Eigen::Matrix<double, 3, 4>> &Rts);

// Point minimizing the squared distance to all rays
// Closed for solution from
//   Srikumar Ramalingam, Suresh K. Lodha and Peter Sturm
//   "A generic structure-from-motion framework"
//   CVIU 2006
template <class T>
Eigen::Matrix<T, 3, 1> TriangulateBearingsMidpointSolve(
    const Eigen::Matrix<T, -1, 3> &centers,
    const Eigen::Matrix<T, -1, 3> &bearings) {
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

namespace geometry {

std::pair<bool, Eigen::Vector3d> TriangulateBearingsDLT(
    const std::vector<Eigen::Matrix<double, 3, 4>> &Rts,
    const Eigen::Matrix<double, -1, 3> &bearings, double threshold,
    double min_angle);

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

  const T eps = T(1e-30);
  const T det = A.determinant();
  if (abs(det) < eps) {
    return std::make_pair(false, Eigen::Matrix<T, 3, 1>());
  }
  const auto lambdas = A.inverse() * b;
  const auto x1 = centers.row(0) + lambdas[0] * bearings.row(0);
  const auto x2 = centers.row(1) + lambdas[1] * bearings.row(1);
  return std::make_pair(true, (x1 + x2) / T(2.0));
};

std::vector<std::pair<bool, Eigen::Vector3d>>
TriangulateTwoBearingsMidpointMany(
    const Eigen::Matrix<double, -1, 3> &bearings1,
    const Eigen::Matrix<double, -1, 3> &bearings2,
    const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation);

std::pair<bool, Eigen::Vector3d> TriangulateBearingsMidpoint(
    const Eigen::Matrix<double, -1, 3> &centers,
    const Eigen::Matrix<double, -1, 3> &bearings,
    const std::vector<double> &threshold_list, double min_angle);

}  // namespace geometry
