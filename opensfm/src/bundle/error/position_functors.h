#pragma once

#include <bundle/error/error_utils.h>
#include <foundation/types.h>

#include <Eigen/Eigen>

namespace bundle {

constexpr int FUNCTOR_NOT_SET = -1;

struct ShotPositionFunctor {
  ShotPositionFunctor() = default;
  explicit ShotPositionFunctor(int rig_instance_index, int rig_camera_index)
      : rig_instance_index_(rig_instance_index),
        rig_camera_index_(rig_camera_index) {}

  template <typename T>
  Vec3<T> operator()(T const* const* p) const {
    const T* const rig_instance = p[rig_instance_index_];
    const bool has_rig_camera =
        rig_camera_index_ != FUNCTOR_NOT_SET && p[rig_camera_index_] != nullptr;

    Vec3<T> instance_position =
        Eigen::Map<const Vec3<T> >(rig_instance + Pose::Parameter::TX);
    if (has_rig_camera) {
      const T* const rig_camera = p[rig_camera_index_];
      Vec3<T> rig_camera_center = Vec3<T>::Zero();
      ceres::AngleAxisRotatePoint(rig_instance + Pose::Parameter::RX,
                                  rig_camera + Pose::Parameter::TX,
                                  rig_camera_center.data());
      instance_position += rig_camera_center;
    }
    return instance_position;
  }
  const int rig_instance_index_{FUNCTOR_NOT_SET};
  const int rig_camera_index_{FUNCTOR_NOT_SET};
};

struct ShotRotationFunctor {
  ShotRotationFunctor() = default;
  explicit ShotRotationFunctor(int rig_instance_index, int rig_camera_index)
      : rig_instance_index_(rig_instance_index),
        rig_camera_index_(rig_camera_index) {}

  template <typename T>
  Vec3<T> operator()(T const* const* p) const {
    const T* const rig_instance = p[rig_instance_index_];
    const bool has_rig_camera =
        rig_camera_index_ != FUNCTOR_NOT_SET && p[rig_camera_index_] != nullptr;

    const Eigen::Map<const Vec3<T> > instance_rotation(rig_instance +
                                                       Pose::Parameter::RX);
    if (has_rig_camera) {
      const T* const rig_camera = p[rig_camera_index_];
      const Eigen::Map<const Vec3<T> > camera_rotation(rig_camera +
                                                       Pose::Parameter::RX);
      return MultRotations(instance_rotation.eval(), camera_rotation.eval());
    } else {
      return instance_rotation.eval();
    }
  }
  const int rig_instance_index_{FUNCTOR_NOT_SET};
  const int rig_camera_index_{FUNCTOR_NOT_SET};
};

struct PointPositionShotFunctor {
  PointPositionShotFunctor() = default;
  PointPositionShotFunctor(int rig_instance_index, int rig_camera_index,
                           int scale_index, int point_index)
      : rig_instance_index_(rig_instance_index),
        rig_camera_index_(rig_camera_index),
        scale_index_(scale_index),
        point_index_(point_index) {}

  template <typename T>
  Vec3<T> operator()(T const* const* p) const {
    const T* const rig_instance = p[rig_instance_index_];
    const T* const rig_camera =
        rig_camera_index_ != FUNCTOR_NOT_SET ? p[rig_camera_index_] : nullptr;
    const T* const point = p[point_index_];
    const T* const scale = p[scale_index_];

    Vec3<T> camera_point = Vec3<T>::Zero();
    WorldToCameraCoordinatesRig(scale, rig_instance, rig_camera, point,
                                camera_point.data());
    return camera_point;
  }

  const int rig_instance_index_{FUNCTOR_NOT_SET};
  const int rig_camera_index_{FUNCTOR_NOT_SET};
  const int scale_index_{FUNCTOR_NOT_SET};
  const int point_index_{FUNCTOR_NOT_SET};
};

struct PointPositionWorldFunctor {
  PointPositionWorldFunctor() = default;
  explicit PointPositionWorldFunctor(int index) : index_(index) {}

  template <typename T>
  Vec3<T> operator()(T const* const* p) const {
    const T* const point = p[index_];
    Eigen::Map<const Vec3<T> > p_world(point);
    return p_world;
  }

  const int index_{FUNCTOR_NOT_SET};
};
}  // namespace bundle
