#pragma once

#include <foundation/types.h>

#include <initializer_list>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace bundle {

template <class T>
Vec3<T> MultRotations(const Vec3<T>& R1, const Vec3<T>& R2) {
  T qR1[4], qR2[4], qResult[4];
  ceres::AngleAxisToQuaternion(R1.data(), qR1);
  ceres::AngleAxisToQuaternion(R2.data(), qR2);
  ceres::QuaternionProduct(qR1, qR2, qResult);

  Vec3<T> result;
  ceres::QuaternionToAngleAxis(qResult, result.data());
  return result;
}

template <class T>
Vec3<T> MultRotations(const Vec3<T>& R1, const Vec3<T>& R2, const Vec3<T>& R3) {
  T qR1[4], qR2[4], qR3[4], qR1R2[4], qResult[4];
  ceres::AngleAxisToQuaternion(R1.data(), qR1);
  ceres::AngleAxisToQuaternion(R2.data(), qR2);
  ceres::AngleAxisToQuaternion(R3.data(), qR3);
  ceres::QuaternionProduct(qR1, qR2, qR1R2);
  ceres::QuaternionProduct(qR1R2, qR3, qResult);

  Vec3<T> result;
  ceres::QuaternionToAngleAxis(qResult, result.data());
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
Vec3<T> WorldToCamera(const Vec3<T>& R, const Vec3<T>& c, const Vec3<T>& x) {
  return RotatePoint((-R).eval(), (x - c).eval());
}

/* apply a similarity transform of scale s, rotation R and translation t to some
 * point x as s * R * x + t */
template <class T>
Vec3<T> ApplySimilarity(const T& s, const Vec3<T>& R, const Vec3<T>& t,
                        const Vec3<T>& x) {
  return RotatePoint((-R).eval(), (s * x - t).eval());
}
}  // namespace bundle
