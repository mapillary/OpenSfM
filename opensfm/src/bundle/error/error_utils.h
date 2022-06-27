#pragma once

#include <bundle/data/pose.h>
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

/* bring a point into shot coordinates as :
x_shot = shot_scale * shot_rotation(t) * (x_world - shot_center)
*/
template <typename T>
void WorldToLocal(const T* scale, const T* const shot, const T world_point[3],
                  T camera_point[3]) {
  const T pt[3] = {scale[0] * world_point[0] - shot[Pose::Parameter::TX],
                   scale[0] * world_point[1] - shot[Pose::Parameter::TY],
                   scale[0] * world_point[2] - shot[Pose::Parameter::TZ]};
  const T Rt[3] = {-shot[Pose::Parameter::RX], -shot[Pose::Parameter::RY],
                   -shot[Pose::Parameter::RZ]};
  ceres::AngleAxisRotatePoint(Rt, pt, camera_point);
}
template <typename T>
void WorldToLocal(const T* const shot, const T world_point[3],
                  T camera_point[3]) {
  const T scale_one = T(1.0);
  WorldToLocal(&scale_one, shot, world_point, camera_point);
}

/* bring a point into shot coordinates as  :
x_shot = rig_camera_rotation(t) * (rig_instance_scale * rig_instance_rotation(t)
* (x_world - rig_instance_center) - rig_camera__center)
*/
template <typename T>
void WorldToCameraCoordinatesRig(const T* scale, const T* const rig_instance,
                                 const T* const rig_camera,
                                 const T world_point[3], T camera_point[3]) {
  if (rig_camera) {
    T instance_point[3];
    WorldToLocal(scale, rig_instance, world_point, instance_point);
    const T scale_one = T(1.0);
    WorldToLocal(&scale_one, rig_camera, instance_point, camera_point);
  } else {
    WorldToLocal(scale, rig_instance, world_point, camera_point);
  }
}

template <typename T>
T DiffBetweenAngles(const T a, const T b) {
  T d = a - b;
  if (d > T(M_PI)) {
    return d - T(2 * M_PI);
  } else if (d < -T(M_PI)) {
    return d + T(2 * M_PI);
  } else {
    return d;
  }
}

}  // namespace bundle
