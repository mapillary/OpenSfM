#pragma once

#include <foundation/numeric.h>
#include <Eigen/Eigen>
#include <complex>
#include <iostream>


Eigen::Matrix3d RotationMatrixAroundAxis(const double cos_theta, const double sin_theta, const Eigen::Vector3d& v);

// Implements "An Efficient Algebraic Solution to the 
// Perspective-Three-Point Problem" from Ke and al.
template <class IT>
std::vector<Eigen::Matrix<double, 3, 4>> AbsolutePoseThreePoints(IT begin, IT end) {
  const auto b1 = begin->first;
  const auto b2 = (begin+1)->first;
  const auto b3 = (begin+2)->first;
  const auto p1 = begin->second;
  const auto p2 = (begin+1)->second;
  const auto p3 = (begin+2)->second;

  // Compute k1, k2 and k3
  const auto k1 = (p1-p2).normalized();
  const auto k3 = (b1.cross(b2)).normalized();
  const auto k2 = (k1.cross(k3)).normalized();

  // Compute ui and vi for i = 1, 2
  const auto u1 = p1 - p3;
  const auto u2 = p2 - p3;
  const auto v1 = b1.cross(b3);
  const auto v2 = b2.cross(b3);

  // Compute sigma and k3"
  const auto u1_k1 = u1.cross(k1);
  const auto sigma = u1_k1.norm();
  const auto k3_second = u1_k1/sigma;

  // Compute fij's
  const auto k3_b3 = k3.dot(b3);
  const auto b1_b2 = b1.cross(b2).norm();
  const auto f11 = sigma*k3_b3;
  const auto f21 = sigma*b1.dot(b2)*k3_b3;
  const auto f22 = sigma*k3_b3*b1_b2;
  const auto f13 = sigma*v1.dot(k3);
  const auto f23 = sigma*v2.dot(k3);
  const auto f24 = u2.dot(k1)*k3_b3*b1_b2;
  const auto f15 = -u1.dot(k1)*k3_b3;
  const auto f25 = -u2.dot(k1)*b1.dot(b2)*k3_b3;

  // Compute gi's
  const auto g1 = f13*f22;
  const auto g2 = f13*f25-f15*f23;
  const auto g3 = f11*f23-f13*f21;
  const auto g4 = -f13*f24;
  const auto g5 = f11*f22;
  const auto g6 = f11*f25-f15*f21;
  const auto g7 = -f15*f24;

  // Solve for cost(theta) by expressing the determinant^2 of (37)
  const auto alpha4 = SQUARE(g5)+SQUARE(g1)+SQUARE(g3);
  const auto alpha3 = 2.0*(g5*g6 + g1*g2 + g3*g4);
  const auto alpha2 = SQUARE(g6) + 2.0*g5*g7 + SQUARE(g2) + SQUARE(g4) - SQUARE(g1) - SQUARE(g3);
  const auto alpha1 = 2.0*(g6*g7 - g1*g2 - g3*g4);
  const auto alpha0 = SQUARE(g7)-SQUARE(g2)-SQUARE(g4);

  std::cout << alpha0 << "\t" << alpha1 << "\t" << alpha2 << "\t" << alpha3 << "\t" << alpha4 << "\t" << std::endl;
  double coefficients[5] = {alpha0, alpha1, alpha2, alpha3, alpha4};
  double roots[4];
  SolveQuartic(coefficients, roots);
  //RefineQuartic(coefficients, roots);

  Eigen::Matrix3d c_barre, c_barre_barre;
  c_barre << k1, k3_second, k1.cross(k3_second);
  c_barre_barre << b1, k3, b1.cross(k3);

  Eigen::Vector3d e1, e2;
  e1 << 1, 0, 0;
  e2 << 0, 1, 0;

  std::vector<Eigen::Matrix<double, 3, 4>> RTs;

  const double eps = 1e-20;
  for(const auto& root : roots){
    const auto cos_theta_1 = root;
    std::cout << root << std::endl;
    const auto sin_theta_1 = Sign(k3_b3)*std::sqrt(1.0-SQUARE(cos_theta_1));
    const auto t = sin_theta_1/(g5*SQUARE(cos_theta_1) + g6*cos_theta_1 + g7);
    
    const auto cos_theta_3 = t*(g1*cos_theta_1 + g2);
    const auto sin_theta_3 = t*(g3*cos_theta_1 + g4);

    const auto c1 = RotationMatrixAroundAxis(cos_theta_1, sin_theta_1, e1);
    const auto c2 = RotationMatrixAroundAxis(cos_theta_3, sin_theta_3, e2);

    const auto rotation = c_barre*c1*c2*c_barre_barre;
    const auto translation = p3 - (sigma*sin_theta_1)/k3_b3*(rotation*b3);

    std::cout << "******" << std::endl;
    std::cout << translation << std::endl << std::endl;
    std::cout << rotation << std::endl << std::endl;
    std::cout << "******" << std::endl;

    // Rcamera and Tcamera parametrization
    Eigen::Matrix<double, 3, 4> RT;
    RT.block<3, 3>(0, 0) = rotation.transpose();
    RT.block<3, 1>(0, 3) = -rotation.transpose()*translation;
    RTs.push_back(RT);
  }
  return RTs;
}

namespace geometry{
std::vector<Eigen::Matrix<double, 3, 4>> AbsolutePoseThreePoints(
  const Eigen::Matrix<double, -1, 3> &bearings,
  const Eigen::Matrix<double, -1, 3> &points);
}
