#pragma once

#include <initializer_list>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

/* multiply a set of N rotation R1*R2*...Rn-1=R rotations are expected to be
 * angle-axis */
template <class T>
Vec3<T> MultRotations(const Vec3<T>& R1, const Vec3<T>& R...) {
  // work-around to run over variadic
  std::initializer_list<Vec3<T> > rotations = {
      R};

  // hence why we split the variadic with a 1st argument
  Vec4<T> qPrevious_Ri;
  ceres::AngleAxisToQuaternion(R1.data(), qPrevious_Ri.data());

  // accumulate rotations in quaternion space
  for (const auto Ri : rotations) {
    Vec4<T> qRi, qResult;
    ceres::AngleAxisToQuaternion(Ri.data(), qRi.data());
    ceres::QuaternionProduct(qPrevious_Ri.data(), qRi.data(), qResult.data());
    qPrevious_Ri = qResult;
  }

  // back to angle axis
  Vec3<T> result;
  ceres::QuaternionToAngleAxis(qPrevious_Ri.data(), result.data());
  return result;
}

/* apply a rotation R to a vector x as R*x rotations is expected to be
 * angle-axis */
template <typename T>
Vec3<T> RotatePoint(const Vec3<T>& R, const Vec3<T>& x) {
  Vec3<T> rotated;
  ceres::AngleAxisRotatePoint(R.data(), x.data(), rotated.data());
  return rotated;
}

/* bring a point x in the coordinate frame of a camera with rotation and camera
 * center in world coordinates being respectively R and c such : x(camera) =
 * R(t).(x(world) - c) */
template <typename T>
Vec3<T>  WorldToCamera(const Vec3<T>& R, const Vec3<T>& c, const Vec3<T>& x) {
  return RotatePoint((-R).eval(), (x - c).eval());
}

/* apply a similarity transform of scale s, rotation R and translation t to some
 * point x as s * R * x + t */
template <class T>
Vec3<T> ApplySimilarity(const T& s, const Vec3<T>& R, const Vec3<T>& t,
                        const Vec3<T>& x) {
  return RotatePoint((-R).eval(), (s * x - t).eval());
}