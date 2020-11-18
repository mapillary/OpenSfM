#include "../essential.h"

// Multiply two polynomials of degree 1.
Eigen::Matrix<double, -1, 1> o1(const Eigen::Matrix<double, -1, 1> &a,
                                const Eigen::Matrix<double, -1, 1> &b) {
  Eigen::Matrix<double, -1, 1> res = Eigen::Matrix<double, -1, 1>::Zero(20);

  res(coef_xx) = a(coef_x) * b(coef_x);
  res(coef_xy) = a(coef_x) * b(coef_y) + a(coef_y) * b(coef_x);
  res(coef_xz) = a(coef_x) * b(coef_z) + a(coef_z) * b(coef_x);
  res(coef_yy) = a(coef_y) * b(coef_y);
  res(coef_yz) = a(coef_y) * b(coef_z) + a(coef_z) * b(coef_y);
  res(coef_zz) = a(coef_z) * b(coef_z);
  res(coef_x) = a(coef_x) * b(coef_1) + a(coef_1) * b(coef_x);
  res(coef_y) = a(coef_y) * b(coef_1) + a(coef_1) * b(coef_y);
  res(coef_z) = a(coef_z) * b(coef_1) + a(coef_1) * b(coef_z);
  res(coef_1) = a(coef_1) * b(coef_1);

  return res;
}

// Multiply a polynomial of degree 2, a, by a polynomial of degree 1, b.
Eigen::Matrix<double, -1, 1> o2(const Eigen::Matrix<double, -1, 1> &a,
                                const Eigen::Matrix<double, -1, 1> &b) {
  Eigen::Matrix<double, -1, 1> res(20);

  res(coef_xxx) = a(coef_xx) * b(coef_x);
  res(coef_xxy) = a(coef_xx) * b(coef_y) + a(coef_xy) * b(coef_x);
  res(coef_xxz) = a(coef_xx) * b(coef_z) + a(coef_xz) * b(coef_x);
  res(coef_xyy) = a(coef_xy) * b(coef_y) + a(coef_yy) * b(coef_x);
  res(coef_xyz) =
      a(coef_xy) * b(coef_z) + a(coef_yz) * b(coef_x) + a(coef_xz) * b(coef_y);
  res(coef_xzz) = a(coef_xz) * b(coef_z) + a(coef_zz) * b(coef_x);
  res(coef_yyy) = a(coef_yy) * b(coef_y);
  res(coef_yyz) = a(coef_yy) * b(coef_z) + a(coef_yz) * b(coef_y);
  res(coef_yzz) = a(coef_yz) * b(coef_z) + a(coef_zz) * b(coef_y);
  res(coef_zzz) = a(coef_zz) * b(coef_z);
  res(coef_xx) = a(coef_xx) * b(coef_1) + a(coef_x) * b(coef_x);
  res(coef_xy) =
      a(coef_xy) * b(coef_1) + a(coef_x) * b(coef_y) + a(coef_y) * b(coef_x);
  res(coef_xz) =
      a(coef_xz) * b(coef_1) + a(coef_x) * b(coef_z) + a(coef_z) * b(coef_x);
  res(coef_yy) = a(coef_yy) * b(coef_1) + a(coef_y) * b(coef_y);
  res(coef_yz) =
      a(coef_yz) * b(coef_1) + a(coef_y) * b(coef_z) + a(coef_z) * b(coef_y);
  res(coef_zz) = a(coef_zz) * b(coef_1) + a(coef_z) * b(coef_z);
  res(coef_x) = a(coef_x) * b(coef_1) + a(coef_1) * b(coef_x);
  res(coef_y) = a(coef_y) * b(coef_1) + a(coef_1) * b(coef_y);
  res(coef_z) = a(coef_z) * b(coef_1) + a(coef_1) * b(coef_z);
  res(coef_1) = a(coef_1) * b(coef_1);

  return res;
}

// Builds the polynomial constraint matrix M.
Eigen::MatrixXd FivePointsPolynomialConstraints(
    const Eigen::MatrixXd &E_basis) {
  // Build the polynomial form of E (equation (8) in Stewenius et al. [1])
  Eigen::Matrix<double, -1, 1> E[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      E[i][j] = Eigen::Matrix<double, -1, 1>::Zero(20);
      E[i][j](coef_x) = E_basis(3 * i + j, 0);
      E[i][j](coef_y) = E_basis(3 * i + j, 1);
      E[i][j](coef_z) = E_basis(3 * i + j, 2);
      E[i][j](coef_1) = E_basis(3 * i + j, 3);
    }
  }

  // The constraint matrix.
  Eigen::MatrixXd M(10, 20);
  int mrow = 0;

  // Determinant constraint det(E) = 0; equation (19) of Nister [2].
  M.row(mrow++) = o2(o1(E[0][1], E[1][2]) - o1(E[0][2], E[1][1]), E[2][0]) +
                  o2(o1(E[0][2], E[1][0]) - o1(E[0][0], E[1][2]), E[2][1]) +
                  o2(o1(E[0][0], E[1][1]) - o1(E[0][1], E[1][0]), E[2][2]);

  // Cubic singular values constraint.
  // Equation (20).
  Eigen::Matrix<double, -1, 1> EET[3][3];
  for (int i = 0; i < 3; ++i) {    // Since EET is symmetric, we only compute
    for (int j = 0; j < 3; ++j) {  // its upper triangular part.
      if (i <= j) {
        EET[i][j] =
            o1(E[i][0], E[j][0]) + o1(E[i][1], E[j][1]) + o1(E[i][2], E[j][2]);
      } else {
        EET[i][j] = EET[j][i];
      }
    }
  }

  // Equation (21).
  Eigen::Matrix<double, -1, 1>(&L)[3][3] = EET;
  Eigen::Matrix<double, -1, 1> trace =
      0.5 * (EET[0][0] + EET[1][1] + EET[2][2]);
  for (int i = 0; i < 3; ++i) {
    L[i][i] -= trace;
  }

  // Equation (23).
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Eigen::Matrix<double, -1, 1> LEij =
          o2(L[i][0], E[0][j]) + o2(L[i][1], E[1][j]) + o2(L[i][2], E[2][j]);
      M.row(mrow++) = LEij;
    }
  }

  return M;
}

// Gauss--Jordan elimination for the constraint matrix.
bool FivePointsGaussJordan(Eigen::MatrixXd *Mp) {
  Eigen::MatrixXd &M = *Mp;

  // Gauss Elimination.
  for (int i = 0; i < 10; ++i) {
    const auto diagonal = M(i, i);
    if (diagonal == 0.0) {
      return false;
    }
    M.row(i) /= diagonal;
    for (int j = i + 1; j < 10; ++j) {
      const auto elem = M(j, i);
      if (elem == 0.0) {
        return false;
      }
      M.row(j) = M.row(j) / elem - M.row(i);
    }
  }

  // Backsubstitution.
  for (int i = 9; i >= 0; --i) {
    for (int j = 0; j < i; ++j) {
      M.row(j) = M.row(j) - M(j, i) * M.row(i);
    }
  }
  return true;
}

namespace geometry {
std::vector<Eigen::Matrix<double, 3, 3>> EssentialFivePoints(
    const Eigen::Matrix<double, -1, 3> &x1,
    const Eigen::Matrix<double, -1, 3> &x2) {
  if ((x1.cols() != x2.cols()) || (x1.rows() != x2.rows())) {
    throw std::runtime_error("Features matrices have different sizes.");
  }
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> samples(x1.rows());
  for (int i = 0; i < x1.rows(); ++i) {
    samples[i].first = x1.row(i);
    samples[i].second = x2.row(i);
  }
  return ::EssentialFivePoints(samples.begin(), samples.end());
}

std::vector<Eigen::Matrix<double, 3, 3>> EssentialNPoints(
    const Eigen::Matrix<double, -1, 3> &x1,
    const Eigen::Matrix<double, -1, 3> &x2) {
  if ((x1.cols() != x2.cols()) || (x1.rows() != x2.rows())) {
    throw std::runtime_error("Features matrices have different sizes.");
  }
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> samples(x1.rows());
  for (int i = 0; i < x1.rows(); ++i) {
    samples[i].first = x1.row(i);
    samples[i].second = x2.row(i);
  }
  return ::EssentialNPoints(samples.begin(), samples.end());
}

}  // namespace geometry
