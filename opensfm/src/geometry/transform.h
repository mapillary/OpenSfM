#pragma once

#include <foundation/numeric.h>
#include <Eigen/Eigen>

template <class IT>
std::pair<Eigen::Vector3d, Eigen::Vector3d> ComputeAverage(IT begin, IT end) {
  Eigen::Vector3d q_average = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_average = Eigen::Vector3d::Zero();
  for (IT it = begin; it != end; ++it) {
    q_average += it->first;
    p_average += it->second;
  }
  q_average /= (end - begin);
  p_average /= (end - begin);
  return std::make_pair(q_average, p_average);
}

template <class IT>
Eigen::Matrix3d RotationBetweenPoints(IT begin, IT end) {
  const auto averages = ComputeAverage(begin, end);
  Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
  for (IT it = begin; it != end; ++it) {
    const Eigen::Vector3d q = it->first - averages.first;
    const Eigen::Vector3d p = it->second - averages.second;
    M += q * p.transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix3d U = svd.matrixU();
  const Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d rotation = U * V.transpose();
  if (rotation.determinant() < 0.) {
    rotation *= -1.0;
  }
  return rotation;
}
