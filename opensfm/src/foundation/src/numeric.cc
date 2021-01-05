#include <foundation/numeric.h>
#include <foundation/newton_raphson.h>
#include <iostream>

namespace foundation {
Eigen::Matrix3d SkewMatrix(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  SkewMatrixT(v, &m);
  return m;
}

Eigen::Matrix3d ClosestRotationMatrix(const Eigen::Matrix3d& matrix) {
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d rotation = svd.matrixU() * svd.matrixV().transpose();
  if (rotation.determinant() < 0.) {
    rotation *= -1.0;
  }
  return rotation;
}

// taken from https://github.com/sidneycadot/quartic
static std::complex<double> ComplexSqrt(const std::complex<double>& z) {
  return pow(z, 1. / 2.);
}

static std::complex<double> ComplexCbrt(const std::complex<double>& z) {
  return pow(z, 1. / 3.);
}

std::array<double, 4> SolveQuartic(const std::array<double, 5>& coefficients) {
  constexpr double eps = std::numeric_limits<double>::epsilon();
  const double a = std::abs(coefficients[4]) > eps
                       ? coefficients[4]
                       : eps;  // Avoid division by zero
  const double b = coefficients[3] / a;
  const double c = coefficients[2] / a;
  const double d = coefficients[1] / a;
  const double e = coefficients[0] / a;

  const double Q1 = c * c - 3. * b * d + 12. * e;
  const double Q2 = 2. * c * c * c - 9. * b * c * d + 27. * d * d +
                    27. * b * b * e - 72. * c * e;
  const double Q3 = 8. * b * c - 16. * d - 2. * b * b * b;
  const double Q4 = 3. * b * b - 8. * c;

  const std::complex<double> Q5 =
      ComplexCbrt(Q2 / 2. + ComplexSqrt(Q2 * Q2 / 4. - Q1 * Q1 * Q1));
  const std::complex<double> Q6 = (Q1 / Q5 + Q5) / 3.;
  const std::complex<double> Q7 = 2. * ComplexSqrt(Q4 / 12. + Q6);

  std::array<double, 4> roots;
  roots[0] =
      (-b - Q7 - ComplexSqrt(4. * Q4 / 6. - 4. * Q6 - Q3 / Q7)).real() / 4.;
  roots[1] =
      (-b - Q7 + ComplexSqrt(4. * Q4 / 6. - 4. * Q6 - Q3 / Q7)).real() / 4.;
  roots[2] =
      (-b + Q7 - ComplexSqrt(4. * Q4 / 6. - 4. * Q6 + Q3 / Q7)).real() / 4.;
  roots[3] =
      (-b + Q7 + ComplexSqrt(4. * Q4 / 6. - 4. * Q6 + Q3 / Q7)).real() / 4.;
  return roots;
}

std::array<double, 4> RefineQuarticRoots(
    const std::array<double, 5>& coefficients,
    const std::array<double, 4>& roots) {
  constexpr int iterations = 5;
  struct QuarticEval {
    const std::array<double, 5> coefficients_;
    double operator()(double x) const {
      return (((coefficients_[4] * x + coefficients_[3]) * x +
               coefficients_[2]) *
                  x +
              coefficients_[1]) *
                 x +
             coefficients_[0];
    }
    double derivative(double x) const {
      const double x2 = x * x;
      const double x3 = x2 * x;
      return 4.0 * coefficients_[4] * x3 + 3.0 * coefficients_[3] * x2 +
             2.0 * coefficients_[2] * x + coefficients_[1];
    }
  };
  QuarticEval eval_function{coefficients};

  std::array<double, 4> refined_roots = roots;
  for (auto& root : refined_roots) {
    root = NewtonRaphson<QuarticEval, 1, 1, ManualDiff<QuarticEval, 1, 1>>(
        eval_function, root, iterations, 1e-20);
  }
  return refined_roots;
}
}  // namespace foundation
