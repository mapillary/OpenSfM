// Copyright (c) 2011 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#pragma once

#include <foundation/numeric.h>
#include <Eigen/Eigen>
#include <Eigen/SVD>

// In the following code, polynomials are expressed as vectors containing
// their coeficients in the basis of monomials:
//
//   [xxx xxy xyy yyy xxz xyz yyz xzz yzz zzz xx xy yy xz yz zz x y z 1]
//
// Note that there is an error in Stewenius' paper.  In equation (9) they
// propose to use the basis:
//
//   [xxx xxy xxz xyy xyz xzz yyy yyz yzz zzz xx xy xz yy yz zz x y z 1]
//
// But this is not the basis used in the rest of the paper, neither in
// the code they provide.  I (pau) spent 4 hours debugging and reverse
// engineering their code to find the problem. :(
enum {
  coef_xxx,
  coef_xxy,
  coef_xyy,
  coef_yyy,
  coef_xxz,
  coef_xyz,
  coef_yyz,
  coef_xzz,
  coef_yzz,
  coef_zzz,
  coef_xx,
  coef_xy,
  coef_yy,
  coef_xz,
  coef_yz,
  coef_zz,
  coef_x,
  coef_y,
  coef_z,
  coef_1
};

template <class IT, class MAT>
inline void EncodeEpipolarEquation(IT begin, IT end, MAT *A) {
  for (IT it = begin; it != end; ++it) {
    int i = (it - begin);
    const auto x1 = it->first;
    const auto x2 = it->second;
    A->row(i) << x2(0) * x1.transpose(), x2(1) * x1.transpose(),
        x2(2) * x1.transpose();
  }
}

// Compute the nullspace of the linear constraints given by the matches.
template <class IT>
Eigen::MatrixXd FivePointsNullspaceBasis(IT begin, IT end) {
  Eigen::Matrix<double, 9, 9> A;
  A.setZero();  // Make A square until Eigen supports rectangular SVD.
  EncodeEpipolarEquation(begin, end, &A);
  Eigen::JacobiSVD<Eigen::Matrix<double, 9, 9>> svd;
  return svd.compute(A, Eigen::ComputeFullV).matrixV().topRightCorner<9, 4>();
}

// Multiply two polynomials of degree 1.
Eigen::Matrix<double, -1, 1> o1(const Eigen::Matrix<double, -1, 1> &a,
                                const Eigen::Matrix<double, -1, 1> &b);

// Multiply a polynomial of degree 2, a, by a polynomial of degree 1, b.
Eigen::Matrix<double, -1, 1> o2(const Eigen::Matrix<double, -1, 1> &a,
                                const Eigen::Matrix<double, -1, 1> &b);

// Builds the polynomial constraint matrix M.
Eigen::MatrixXd FivePointsPolynomialConstraints(const Eigen::MatrixXd &E_basis);

// Gauss--Jordan elimination for the constraint matrix.
bool FivePointsGaussJordan(Eigen::MatrixXd *Mp);

template <class IT>
std::vector<Eigen::Matrix<double, 3, 3>> EssentialFivePoints(IT begin, IT end) {
  // Step 1: Nullspace exrtraction.
  Eigen::MatrixXd E_basis = FivePointsNullspaceBasis(begin, end);

  // Step 2: Constraint expansion.
  Eigen::MatrixXd M = FivePointsPolynomialConstraints(E_basis);

  // Step 3: Gauss-Jordan elimination.
  if (!FivePointsGaussJordan(&M)) {
    return std::vector<Eigen::Matrix<double, 3, 3>>();
  }

  // For the next steps, follow the matlab code given in Stewenius et al [1].

  // Build the action matrix.
  Eigen::MatrixXd B = M.topRightCorner<10, 10>();
  Eigen::MatrixXd At = Eigen::MatrixXd::Zero(10, 10);
  At.row(0) = -B.row(0);
  At.row(1) = -B.row(1);
  At.row(2) = -B.row(2);
  At.row(3) = -B.row(4);
  At.row(4) = -B.row(5);
  At.row(5) = -B.row(7);
  At(6, 0) = 1;
  At(7, 1) = 1;
  At(8, 3) = 1;
  At(9, 6) = 1;

  // Compute the solutions from action matrix's eigenvectors.
  Eigen::EigenSolver<Eigen::MatrixXd> es(At);
  typedef Eigen::EigenSolver<Eigen::MatrixXd>::EigenvectorsType Matc;
  Matc V = es.eigenvectors();
  Matc solutions(4, 10);
  solutions.row(0) = V.row(6).array() / V.row(9).array();
  solutions.row(1) = V.row(7).array() / V.row(9).array();
  solutions.row(2) = V.row(8).array() / V.row(9).array();
  solutions.row(3).setOnes();

  // Get the ten candidate E matrices in vector form.
  Matc Evec = E_basis * solutions;

  // Build the essential matrices for the real solutions.
  std::vector<Eigen::Matrix<double, 3, 3>> Es;
  Es.reserve(10);
  for (int s = 0; s < 10; ++s) {
    Evec.col(s) /= Evec.col(s).norm();
    bool is_real = true;
    for (int i = 0; i < 9; ++i) {
      if (Evec(i, s).imag() != 0) {
        is_real = false;
        break;
      }
    }
    if (is_real) {
      Eigen::Matrix<double, 3, 3> E;
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          E(i, j) = Evec(3 * i + j, s).real();
        }
      }
      Es.push_back(E);
    }
  }
  return Es;
}

template <class IT>
std::vector<Eigen::Matrix<double, 3, 3>> EssentialNPoints(IT begin, IT end) {
  const int count = end - begin;
  Eigen::MatrixXd A(count, 9);
  A.setZero();
  EncodeEpipolarEquation(begin, end, &A);

  Eigen::VectorXd solution;
  std::vector<Eigen::Matrix<double, 3, 3>> Es;
  if (foundation::SolveAX0(A, &solution)) {
    Eigen::Matrix3d E =
        Eigen::Map<Eigen::Matrix3d>(solution.data()).transpose();
    if (count > 8) {
      Eigen::JacobiSVD<Eigen::Matrix3d> USV(
          E, Eigen::ComputeFullU | Eigen::ComputeFullV);

      // Enforce essential matrix constraints
      auto d = USV.singularValues();
      const auto a = d[0];
      const auto b = d[1];
      d << (a + b) / 2., (a + b) / 2., 0.0;
      E = USV.matrixU() * d.asDiagonal() * USV.matrixV().transpose();
    }
    Es.push_back(E);
  }
  return Es;
}

namespace geometry {
std::vector<Eigen::Matrix<double, 3, 3>> EssentialFivePoints(
    const Eigen::Matrix<double, -1, 3> &x1,
    const Eigen::Matrix<double, -1, 3> &x2);

std::vector<Eigen::Matrix<double, 3, 3>> EssentialNPoints(
    const Eigen::Matrix<double, -1, 3> &x1,
    const Eigen::Matrix<double, -1, 3> &x2);
}  // namespace geometry
