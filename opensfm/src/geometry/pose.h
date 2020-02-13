#pragma once

#include <Eigen/Eigen>
#include <Eigen/SVD>
#include "triangulation.h"

template <class IT>
Eigen::Matrix<double, 3, 4> RelativePoseFromEssential(
    const Eigen::Matrix3d& essential, IT begin, IT end) {

  Eigen::JacobiSVD<Eigen::Matrix3d> SVD(essential, Eigen::ComputeFullV | Eigen::ComputeFullU);
  Eigen::Matrix3d U = SVD.matrixU();
  Eigen::Matrix3d Vt = SVD.matrixV().transpose();

  // Last column of U is undetermined since d = (a a 0).
  if (U.determinant() < 0) {
    U.col(2) *= -1;
  }
  // Last row of Vt is undetermined since d = (a a 0).
  if (Vt.determinant() < 0) {
    Vt.row(2) *= -1;
  }

  Eigen::Matrix3d W;
  W << 0, -1, 0, 
       1, 0, 0, 
       0, 0, 1;

  Eigen::Matrix<double, 3, 2> bearings;
  Eigen::Matrix<double, 3, 2> centers;
  centers.col(0) << Eigen::Vector3d::Zero();

  auto best_decomposition = std::make_pair(0, Eigen::Matrix<double, 3, 4>());
  for (int i = 0; i < 2; ++i) {
    Eigen::Vector3d translation;
    if (i == 0) {
      translation = U.col(2);
    } else {
      translation = -U.col(2);
    }
    translation.normalize();
    for (int j = 0; j < 2; ++j) {
      Eigen::Matrix3d rotation;
      if (j == 0) {
        rotation = U * W * Vt;
      } else {
        rotation = U * W.transpose() * Vt;
      }
      centers.col(1) << -rotation.transpose()*translation;

      int are_in_front = 0;
      for (IT it = begin; it != end; ++it) {
        bearings.col(0) << it->first;
        bearings.col(1) << rotation.transpose()*it->second;
        const auto point = geometry::TriangulateBearingsMidpointSolve(centers, bearings);
        const bool is_in_front =
            bearings.col(0).dot(point) > 0.0 &&
            bearings.col(1).dot(rotation * point + translation) > 0.0;
        are_in_front += is_in_front;

        const auto maximum_possible = are_in_front + (end - it);
        if (maximum_possible < best_decomposition.first) {
          break;
        }
      }

      if (are_in_front > best_decomposition.first) {
        Eigen::Matrix<double, 3, 4> RT;
        RT.block<3, 3>(0, 0) = rotation;
        RT.block<3, 1>(0, 3) = translation;
        best_decomposition = std::make_pair(are_in_front, RT);
      }
    }
  }
  return best_decomposition.second;
}

namespace geometry {
Eigen::Matrix<double, 3, 4> RelativePoseFromEssential(
    const Eigen::Matrix3d& essential,
    const Eigen::Matrix<double, -1, 3> &x1,
    const Eigen::Matrix<double, -1, 3> &x2);
}