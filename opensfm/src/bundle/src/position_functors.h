#pragma once

#include "error_utils.h"

#include <Eigen/Eigen>


struct ShotPositionShotParam{
  ShotPositionShotParam() = default;
  ShotPositionShotParam(int index) : index_(index) {}

  template <typename T>
  Vec3<T> operator()(T const* const* p)const{
    const T* const shot = p[index_];
    Eigen::Map< const Vec3<T> > t(shot + BA_SHOT_TX);
    return t;
  }
  const int index_{-1};
};

struct ShotPositionWorldParam{
  ShotPositionWorldParam() = default;
  ShotPositionWorldParam(int index) : index_(index) {}

  template <typename T>
  Vec3<T> operator()(T const* const* p)const{
    const T* const shot = p[index_];
    Eigen::Map< const Vec3<T> > t(shot + BA_SHOT_TX);
    return t;
  }
  const int index_{-1};
};

struct PointPositionScaledShot{
  PointPositionScaledShot() = default;
  PointPositionScaledShot(int shot_index, int scale_index, int point_index)
      : shot_index_(shot_index),
        scale_index_(scale_index),
        point_index_(point_index) {}

  template <typename T>
  Vec3<T> operator()(T const* const* p)const{
    const T* const shot = p[shot_index_];
    Eigen::Map< const Vec3<T> > R(shot + BA_SHOT_RX);
    Eigen::Map< const Vec3<T> > t(shot + BA_SHOT_TX);

    const T* const point = p[point_index_];
    Eigen::Map< const Vec3<T> > p_world(point);

    const T* const scale = p[scale_index_];

    return ApplySimilarity(scale[0], R.eval(), t.eval(), p_world.eval());
  }

  const int shot_index_{-1};
  const int scale_index_{-1};
  const int point_index_{-1};
};

struct PointPositionWorld{
  PointPositionWorld() = default;
  PointPositionWorld(int index)
      : index_(index) {}

  template <typename T>
  Vec3<T> operator()(T const* const* p)const{
    const T* const point = p[index_];
    Eigen::Map< const Vec3<T> > p_world(point);
    return p_world;
  }

  const int index_{-1};
};
