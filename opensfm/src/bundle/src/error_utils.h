#pragma once

#include <initializer_list>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

/* helper to entangle Eigen stuff and get base type */
template <class T>
using MatrixType = typename Eigen::MatrixBase<T>::Scalar;

/* multiply a set of N rotation R1*R2*...Rn-1=R rotations are expected to be
 * angle-axis*/
template <class T1, class... T>
Eigen::Matrix<MatrixType<T1>, 3, 1> MultRotations(const T1& R1, T... R) {
  // work-around to run over variadic
  std::initializer_list<Eigen::Matrix<MatrixType<T1>, 3, 1> > rotations = {
      R...};

  // hence why we split the variadic with a 1st argument
  Eigen::Array<MatrixType<T1>, 4, 1> qPrevious_Ri;
  ceres::AngleAxisToQuaternion(R1.data(), qPrevious_Ri.data());

  // accumulate rotations in quaternion space
  for (const auto Ri : rotations) {
    Eigen::Array<MatrixType<T1>, 4, 1> qRi, qResult;
    ceres::AngleAxisToQuaternion(Ri.data(), qRi.data());
    ceres::QuaternionProduct(qPrevious_Ri.data(), qRi.data(), qResult.data());
    qPrevious_Ri = qResult;
  }

  // back to angle axis
  Eigen::Matrix<MatrixType<T1>, 3, 1> result;
  ceres::QuaternionToAngleAxis(qPrevious_Ri.data(), result.data());
  return result;
}

/* apply a rotation R to a vector x as R*x rotations is expected to be
 * angle-axis */
template <class T1, class T2>
Eigen::Matrix<MatrixType<T1>, 3, 1> RotatePoint(const T1& R, const T2& x) {
  Eigen::Matrix<MatrixType<T1>, 3, 1> rotated;
  ceres::AngleAxisRotatePoint(R.data(), x.data(), rotated.data());
  return rotated;
}

/* bring a point x in the coordinate frame of a camera with rotation and camera
 * center in world coordinates being respectively R and c such : x(camera) =
 * R(t).(x(world) - c) */
template <typename T1, typename T2, typename T3>
Eigen::Matrix<MatrixType<T3>, 3, 1>  WorldToCamera(const T1& R, const T2& c, const T3& x) {
  return RotatePoint((-R).eval(), (x - c).eval());
}

/* apply a similarity transform of scale s, rotation R and translation t to some
 * point x as s * R * x + t */
template <class T1, class T2, class T3, class T4>
Eigen::Matrix<MatrixType<T2>, 3, 1> ApplySimilarity(const T1& s, const T2& R,
                                                    const T3& t, const T4& x) {
  return RotatePoint((-R).eval(), (s * x - t).eval());
}