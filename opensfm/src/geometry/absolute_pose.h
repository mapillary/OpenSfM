#pragma once

#include <foundation/numeric.h>

#include <Eigen/Eigen>

#include <complex>

// taken from https://github.com/sidneycadot/quartic
static std::complex<double> complex_sqrt(const std::complex<double> & z)
{
    return pow(z, 1. / 2.);
}

static std::complex<double> complex_cbrt(const std::complex<double> & z)
{
    return pow(z, 1. / 3.);
}

void solve_quartic(const std::complex<double> coefficients[5], std::complex<double> roots[4])
{
    // The algorithm below was derived by solving the quartic in Mathematica, and simplifying the resulting expression by hand.

    const std::complex<double> a = coefficients[4];
    const std::complex<double> b = coefficients[3] / a;
    const std::complex<double> c = coefficients[2] / a;
    const std::complex<double> d = coefficients[1] / a;
    const std::complex<double> e = coefficients[0] / a;

    const std::complex<double> Q1 = c * c - 3. * b * d + 12. * e;
    const std::complex<double> Q2 = 2. * c * c * c - 9. * b * c * d + 27. * d * d + 27. * b * b * e - 72. * c * e;
    const std::complex<double> Q3 = 8. * b * c - 16. * d - 2. * b * b * b;
    const std::complex<double> Q4 = 3. * b * b - 8. * c;

    const std::complex<double> Q5 = complex_cbrt(Q2 / 2. + complex_sqrt(Q2 * Q2 / 4. - Q1 * Q1 * Q1));
    const std::complex<double> Q6 = (Q1 / Q5 + Q5) / 3.;
    const std::complex<double> Q7 = 2. * complex_sqrt(Q4 / 12. + Q6);

    roots[0] = (-b - Q7 - complex_sqrt(4. * Q4 / 6. - 4. * Q6 - Q3 / Q7)) / 4.;
    roots[1] = (-b - Q7 + complex_sqrt(4. * Q4 / 6. - 4. * Q6 - Q3 / Q7)) / 4.;
    roots[2] = (-b + Q7 - complex_sqrt(4. * Q4 / 6. - 4. * Q6 + Q3 / Q7)) / 4.;
    roots[3] = (-b + Q7 + complex_sqrt(4. * Q4 / 6. - 4. * Q6 + Q3 / Q7)) / 4.;
}

template< class T>
T sign(const T& a){
  if(a < T(0)){
    return T(-1);
  }
  else{
    return T(1);
  }
}

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
  const auto a = f11*f22;
  const auto b = f25*f11-f15*f21;
  const auto c = -f24*f15;
  const auto alpha4 = a*a;
  const auto alpha3 = 2.0*a*b;
  const auto alpha2 = 2.0*a*c+b*b;
  const auto alpha1 = 2.0*b*c;
  const auto alpha0 = c*c;

  std::complex<double> coefficients[5] = {alpha0, alpha1, alpha2, alpha3, alpha4};
  std::complex<double> roots[4];
  solve_quartic(coefficients, roots);

  const double eps = 1e-20;
  for(const auto& root : roots)
    if(root.imag() > eps){
      continue;
    }
    const auto cos_theta_1 = root.real();
    const auto sin_theta_1 = sign(k3_b3)*std::sqrt(1.0-SQUARE(std::cos(cos_theta)));
    const auto t = sin_theta_1/(g5*cos_theta_1*cos_theta_1 + g6*cos_theta_1 + g7);
    const auto cos_theta_3 = t*(g1*cos_theta_1 + g2);
    const auto sin_theta_3 = t*(g3*cos_theta_1 + g4);
    const auto Eigen::Matrix3d C1, C2;

    Eigen::Matrix3d c_barre, c_barre_barre;
    Eigen::Matrix3d rotation = c_barre*C1*C2*c_barre_barre;

  }
}
}