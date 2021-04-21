#pragma once

#include <foundation/numeric.h>
#include <geometry/transform.h>
#include <Eigen/Eigen>
#include <complex>
#include <iostream>

Eigen::Matrix3d RotationMatrixAroundAxis(const double cos_theta,
                                         const double sin_theta,
                                         const Eigen::Vector3d &v);

// Implements "An Efficient Algebraic Solution to the
// Perspective-Three-Point Problem" from Ke and al.
template <class IT>
std::vector<Eigen::Matrix<double, 3, 4>> AbsolutePoseThreePoints(IT begin,
                                                                 IT end) {
  std::vector<Eigen::Matrix<double, 3, 4>> RTs;

  const Eigen::Vector3d b1 = begin->first;
  const Eigen::Vector3d b2 = (begin + 1)->first;
  const Eigen::Vector3d b3 = (begin + 2)->first;
  const Eigen::Vector3d p1 = begin->second;
  const Eigen::Vector3d p2 = (begin + 1)->second;
  const Eigen::Vector3d p3 = (begin + 2)->second;

  // Compute k1, k2 and k3
  const Eigen::Vector3d k1 = (p1 - p2).normalized();
  const Eigen::Vector3d k3 = (b1.cross(b2)).normalized();
  const Eigen::Vector3d k2 = (k1.cross(k3)).normalized();

  // Compute ui and vi for i = 1, 2
  const Eigen::Vector3d u1 = p1 - p3;
  const Eigen::Vector3d u2 = p2 - p3;
  const Eigen::Vector3d v1 = b1.cross(b3);
  const Eigen::Vector3d v2 = b2.cross(b3);

  // Compute sigma and k3"
  const Eigen::Vector3d u1_k1 = u1.cross(k1);
  const auto sigma = u1_k1.norm();
  if (sigma == 0.0) {
    return RTs;
  }
  const Eigen::Vector3d k3_second = u1_k1 / sigma;

  // Compute fij's
  const auto k3_b3 = k3.dot(b3);
  if (k3_b3 == 0.0) {
    return RTs;
  }
  const auto b1_b2 = b1.cross(b2).norm();
  const auto f11 = sigma * k3_b3;
  const auto f21 = sigma * b1.dot(b2) * k3_b3;
  const auto f22 = sigma * k3_b3 * b1_b2;
  const auto f13 = sigma * v1.dot(k3);
  const auto f23 = sigma * v2.dot(k3);
  const auto f24 = u2.dot(k1) * k3_b3 * b1_b2;
  const auto f15 = -u1.dot(k1) * k3_b3;
  const auto f25 = -u2.dot(k1) * b1.dot(b2) * k3_b3;

  // Compute gi's
  const auto g1 = f13 * f22;
  const auto g2 = f13 * f25 - f15 * f23;
  const auto g3 = f11 * f23 - f13 * f21;
  const auto g4 = -f13 * f24;
  const auto g5 = f11 * f22;
  const auto g6 = f11 * f25 - f15 * f21;
  const auto g7 = -f15 * f24;

  // Solve for cost(theta) by expressing the determinant^2 of (37)
  const auto alpha4 = SQUARE(g5) + SQUARE(g1) + SQUARE(g3);
  const auto alpha3 = 2.0 * (g5 * g6 + g1 * g2 + g3 * g4);
  const auto alpha2 = SQUARE(g6) + 2.0 * g5 * g7 + SQUARE(g2) + SQUARE(g4) -
                      SQUARE(g1) - SQUARE(g3);
  const auto alpha1 = 2.0 * (g6 * g7 - g1 * g2 - g3 * g4);
  const auto alpha0 = SQUARE(g7) - SQUARE(g2) - SQUARE(g4);

  std::array<double, 5> coefficients = {alpha0, alpha1, alpha2, alpha3, alpha4};
  std::array<double, 4> roots = foundation::SolveQuartic(coefficients);
  roots = foundation::RefineQuarticRoots(coefficients, roots);

  Eigen::Matrix3d c_barre, c_barre_barre;
  c_barre << k1, k3_second, k1.cross(k3_second);
  c_barre_barre << b1, k3, b1.cross(k3);
  c_barre_barre.transposeInPlace();

  Eigen::Vector3d e1, e2;
  e1 << 1, 0, 0;
  e2 << 0, 1, 0;

  constexpr double eps = 1e-20;
  for (const auto &root : roots) {
    const auto cos_theta_1 = root;
    const auto sin_theta_1 =
        foundation::Sign(k3_b3) * std::sqrt(1.0 - SQUARE(cos_theta_1));
    const auto t =
        sin_theta_1 / (g5 * SQUARE(cos_theta_1) + g6 * cos_theta_1 + g7);

    const auto cos_theta_3 = t * (g1 * cos_theta_1 + g2);
    const auto sin_theta_3 = t * (g3 * cos_theta_1 + g4);

    const Eigen::Matrix3d c1 =
        RotationMatrixAroundAxis(cos_theta_1, sin_theta_1, e1);
    const Eigen::Matrix3d c2 =
        RotationMatrixAroundAxis(cos_theta_3, sin_theta_3, e2);

    const Eigen::Matrix3d rotation =
        foundation::ClosestRotationMatrix(c_barre * c1 * c2 * c_barre_barre);
    const Eigen::Vector3d translation =
        p3 - (sigma * sin_theta_1) / k3_b3 * (rotation * b3);

    // Rcamera and Tcamera parametrization
    Eigen::Matrix<double, 3, 4> RT;
    RT.block<3, 3>(0, 0) = rotation.transpose();
    RT.block<3, 1>(0, 3) = -rotation.transpose() * translation;
    RTs.push_back(RT);
  }
  return RTs;
}

template <class IT>
Eigen::Vector3d TranslationBetweenPoints(IT begin, IT end,
                                         const Eigen::Matrix3d &rotation) {
  Eigen::Matrix3d F1 = Eigen::Matrix3d::Zero();
  Eigen::Vector3d F2 = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  for (IT it = begin; it != end; ++it) {
    const auto v = it->first;
    const Eigen::Matrix3d F = (v * v.transpose()) / (v.dot(v));
    F1 += F;
    F2 += (F - identity) * rotation * it->second;
  }
  F2 /= (end - begin);
  F1 /= (end - begin);

  return (identity - F1).inverse() * F2;
}

// Implements "Fast and Globally Convergent Pose
// Estimation from Video Images" from Lu and al.
template <class IT>
Eigen::Matrix<double, 3, 4> AbsolutePoseNPoints(IT begin, IT end) {
  // Initialize by compute s, R and t using Horn's method between rays and
  // points
  const auto averages = ComputeAverage(begin, end);
  double s_num = 0., s_denum = 0.;
  for (IT it = begin; it != end; ++it) {
    const auto q = it->first - averages.first;
    const auto p = it->second - averages.second;
    s_num += SQUARE(p.norm());
    s_denum += SQUARE(q.norm());
  }
  const double scale = std::sqrt(s_num / s_denum);
  Eigen::Matrix3d rotation = RotationBetweenPoints(begin, end);
  Eigen::Vector3d translation =
      scale * averages.first - rotation * averages.second;

  const double tolerance = 1e-7;
  const int max_iterations = 100;
  for (int i = 0; i < max_iterations; ++i) {
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> current_points;
    for (IT it = begin; it != end; ++it) {
      const auto v = it->first;
      const Eigen::Matrix3d F = (v * v.transpose()) / (v.dot(v));
      const auto p = it->second;
      const auto q = F * (rotation * p + translation);
      current_points.push_back(std::make_pair(q, p));
    }
    rotation =
        RotationBetweenPoints(current_points.begin(), current_points.end());
    const auto new_translation = TranslationBetweenPoints(begin, end, rotation);
    const auto rel_delta =
        (new_translation - translation).norm() / translation.norm();
    if (rel_delta < tolerance) {
      break;
    }
    translation = new_translation;
  }
  // Rcamera and Tcamera parametrization
  Eigen::Matrix<double, 3, 4> RT;
  RT.block<3, 3>(0, 0) = rotation;
  RT.block<3, 1>(0, 3) = translation;
  return RT;
}

template <class IT>
Eigen::Vector3d AbsolutePoseNPointsKnownRotation(IT begin, IT end) {
  return TranslationBetweenPoints(begin, end, Eigen::Matrix3d::Identity());
}

namespace geometry {
std::vector<Eigen::Matrix<double, 3, 4>> AbsolutePoseThreePoints(
    const Eigen::Matrix<double, -1, 3> &bearings,
    const Eigen::Matrix<double, -1, 3> &points);

Eigen::Matrix<double, 3, 4> AbsolutePoseNPoints(
    const Eigen::Matrix<double, -1, 3> &bearings,
    const Eigen::Matrix<double, -1, 3> &points);

Eigen::Vector3d AbsolutePoseNPointsKnownRotation(
    const Eigen::Matrix<double, -1, 3> &bearings,
    const Eigen::Matrix<double, -1, 3> &points);
}  // namespace geometry
