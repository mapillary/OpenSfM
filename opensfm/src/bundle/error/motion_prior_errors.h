#pragma once

#include <bundle/data/pose.h>
#include <bundle/error/error_utils.h>
#include <bundle/error/position_functors.h>
#include <foundation/types.h>

#include <Eigen/Eigen>

namespace bundle {

struct LinearMotionError {
  LinearMotionError(double alpha, double position_std_deviation,
                    double orientation_std_deviation)
      : alpha_(alpha),
        position_scale_(1.0 / position_std_deviation),
        orientation_scale_(1.0 / orientation_std_deviation) {}

  template <typename T>
  bool operator()(T const* const* p, T* r) const {
    Vec3<T> R0 = ShotRotationFunctor(shot0_rig_instance_index,
                                     shot0_rig_camera_index)(p);
    Vec3<T> t0 = ShotPositionFunctor(shot0_rig_instance_index,
                                     shot0_rig_camera_index)(p);
    Vec3<T> R1 = ShotRotationFunctor(shot1_rig_instance_index,
                                     shot1_rig_camera_index)(p);
    Vec3<T> t1 = ShotPositionFunctor(shot1_rig_instance_index,
                                     shot1_rig_camera_index)(p);
    Vec3<T> R2 = ShotRotationFunctor(shot2_rig_instance_index,
                                     shot2_rig_camera_index)(p);
    Vec3<T> t2 = ShotPositionFunctor(shot2_rig_instance_index,
                                     shot2_rig_camera_index)(p);

    // Residual have the general form :
    //  op( alpha . op(2, -0), op(0, -1))
    // with - being the opposite
    Eigen::Map<Eigen::Matrix<T, 6, 1> > residual(r);

    // Position residual : op is translation
    const auto eps = T(1e-15);
    const auto t2_t0 = (t2 - t0);
    const auto t1_t0 = (t1 - t0);
    const auto t2_t0_norm = t2_t0.norm();
    const auto t1_t0_norm = t1_t0.norm();
    for (int i = 0; i < 3; ++i) {
      // in case of non-zero speed, use speed ratio as it isn't subject
      //  to collapse when used together with position variance estimation
      if (t2_t0_norm > eps) {
        residual(i) =
            T(position_scale_) * (T(alpha_) - t1_t0_norm / t2_t0_norm);
        // otherwise, use classic difference between speeds, so one can still
        // drag away position if they're coincident
      } else {
        residual(i) = T(position_scale_) * (T(alpha_) * t2_t0(i) - t1_t0(i));
      }
    }

    // Rotation residual : op is rotation
    const Vec3<T> R2_R0t = T(alpha_) * MultRotations(R2.eval(), (-R0).eval());
    const Vec3<T> R0_R1t = MultRotations(R0.eval(), (-R1).eval());
    residual.segment(3, 3) =
        T(orientation_scale_) * MultRotations(R2_R0t.eval(), R0_R1t);
    return true;
  }
  const double alpha_;
  const double position_scale_;
  const double orientation_scale_;

  int shot0_rig_camera_index{FUNCTOR_NOT_SET};
  int shot1_rig_camera_index{FUNCTOR_NOT_SET};
  int shot2_rig_camera_index{FUNCTOR_NOT_SET};
  static constexpr int shot0_rig_instance_index = 0;
  static constexpr int shot1_rig_instance_index = 1;
  static constexpr int shot2_rig_instance_index = 2;
};
}  // namespace bundle
