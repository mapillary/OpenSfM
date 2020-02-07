#pragma once

#define SQUARE(x) ((x)*(x))

template<class MAT, class VEC>
bool SolveAX0(const MAT& A, VEC* solution){
  Eigen::JacobiSVD<MAT> svd(A, Eigen::ComputeFullV);
  const int data_size = A.cols();
  *solution = svd.matrixV().col(data_size-1);

  // Check ratio of smallest eigenvalues for single nullspace
  const double minimum_ratio = 1e3;
  const auto values = svd.singularValues();
  const double ratio = values(data_size-2)/values(data_size-1);

  // Enough constraint and some nullspace will make a solution
  const bool constrained = A.rows() >= A.cols();
  const bool some_nullspace = ratio > minimum_ratio;
  if (constrained && some_nullspace)
    return true;
  else
    return false;
}