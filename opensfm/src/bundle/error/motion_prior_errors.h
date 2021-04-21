#pragma once

#include <bundle/data/pose.h>
#include <bundle/error/error_utils.h>
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
  bool operator()(const T* const shot0, const T* const shot1,
                  const T* const shot2, T* r) const {
    Eigen::Map<const Vec3<T> > R0(shot0 + Pose::Parameter::RX);
    Eigen::Map<const Vec3<T> > t0(shot0 + Pose::Parameter::TX);
    Eigen::Map<const Vec3<T> > R1(shot1 + Pose::Parameter::RX);
    Eigen::Map<const Vec3<T> > t1(shot1 + Pose::Parameter::TX);
    Eigen::Map<const Vec3<T> > R2(shot2 + Pose::Parameter::RX);
    Eigen::Map<const Vec3<T> > t2(shot2 + Pose::Parameter::TX);

    // Residual have the general form :
    //  op( alpha . op(2, -0), op(0, -1))
    // with - being the opposite
    Eigen::Map<Eigen::Matrix<T, 6, 1> > residual(r);

    // Position residual : op is translation
    residual.segment(0, 3) =
        T(position_scale_) * (T(alpha_) * (t2 - t0) + (t0 - t1));

    // Rotation residual : op is rotation
    const Vec3<T> R2_R0t = T(alpha_) * MultRotations(R2.eval(), (-R0).eval());
    const Vec3<T> R0_R1t = MultRotations(R0.eval(), (-R1).eval());
    residual.segment(3, 3) =
        T(position_scale_) * MultRotations(R2_R0t.eval(), R0_R1t);
    return true;
  }
  double alpha_;
  Vec3d acceleration_;
  double position_scale_;
  double orientation_scale_;
};
}  // namespace bundle
