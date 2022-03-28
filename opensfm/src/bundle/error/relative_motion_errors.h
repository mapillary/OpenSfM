#pragma once

#include <bundle/data/bias.h>
#include <bundle/data/pose.h>
#include <bundle/error/error_utils.h>
#include <bundle/error/position_functors.h>
#include <foundation/types.h>

#include <Eigen/Eigen>

namespace bundle {

struct RelativeMotionError {
  RelativeMotionError(const Eigen::VectorXd& observed_Rts,
                      const Eigen::MatrixXd& scale_matrix, bool observed_scale)
      : observed_Rts_(observed_Rts),
        scale_matrix_(scale_matrix),
        observed_scale_(observed_scale) {}

  template <typename T>
  bool operator()(T const* const* p, T* r) const {
    // Get rotation and translation values.
    Vec3<T> Ri =
        ShotRotationFunctor(shot_i_rig_instance_index_, FUNCTOR_NOT_SET)(p);
    Vec3<T> ti =
        ShotPositionFunctor(shot_i_rig_instance_index_, FUNCTOR_NOT_SET)(p);
    Vec3<T> Rj =
        ShotRotationFunctor(shot_j_rig_instance_index_, FUNCTOR_NOT_SET)(p);
    Vec3<T> tj =
        ShotPositionFunctor(shot_j_rig_instance_index_, FUNCTOR_NOT_SET)(p);
    Eigen::Map<VecX<T> > residual(r, Similarity::Parameter::NUM_PARAMS);

    // Compute rotation residual: log( Rij Ri^t Rj)
    const Vec3<T> Rij = observed_Rts_.segment<3>(Pose::Parameter::RX).cast<T>();
    residual.segment(Pose::Parameter::RX, 3) =
        MultRotations(Rij, (-Ri).eval(), Rj.eval());

    // Compute translation residual: tij - sj * Rj^t * (ti - tj)
    const auto scale_i = p[scale_i_index_];
    const auto scale_j = p[scale_j_index_];
    const auto tij = observed_Rts_.segment<3>(Pose::Parameter::TX).cast<T>();
    residual.segment(Pose::Parameter::TX, 3) =
        tij - scale_j[0] * RotatePoint((-Rj).eval(), (ti - tj).eval());

    if (scale_i[0] == T(0.0) || scale_j[0] == T(0.0)) {
      return false;
    }

    if (observed_scale_) {
      const auto Sij = T(observed_Rts_(Similarity::Parameter::SCALE));
      residual(Similarity::Parameter::SCALE) = Sij - scale_j[0] / scale_i[0];
    } else {
      residual(Similarity::Parameter::SCALE) = T(0.);
    }
    residual = scale_matrix_.cast<T>() * residual;
    return true;
  }

  Eigen::VectorXd observed_Rts_;
  Eigen::MatrixXd scale_matrix_;
  static constexpr int shot_i_rig_instance_index_ = 0;
  static constexpr int shot_j_rig_instance_index_ = 1;
  bool observed_scale_;
  int scale_i_index_{2};
  int scale_j_index_{2};
};

struct RelativeRotationError {
  RelativeRotationError(const Vec3d& Rij, const Eigen::Matrix3d& scale_matrix)
      : Rij_(Rij), scale_matrix_(scale_matrix) {}

  template <typename T>
  bool operator()(T const* const* p, T* r) const {
    // Get rotation and translation values.
    Vec3<T> Ri = ShotRotationFunctor(shot_i_rig_instance_index_,
                                     shot_i_rig_camera_index_)(p);
    Vec3<T> Rj = ShotRotationFunctor(shot_j_rig_instance_index_,
                                     shot_j_rig_camera_index_)(p);
    Eigen::Map<Eigen::Matrix<T, 3, 1> > residual(r);

    // Compute rotation residual: log( Rij Ri Rj^t ) -> log( Rij Ri^t Rj)
    const Eigen::Matrix<T, 3, 1> Rij = Rij_.cast<T>();
    residual =
        scale_matrix_.cast<T>() * MultRotations(Rij, (-Ri).eval(), Rj.eval());
    return true;
  }

  Vec3d Rij_;
  Eigen::Matrix3d scale_matrix_;

  int shot_i_rig_camera_index_{FUNCTOR_NOT_SET};
  int shot_j_rig_camera_index_{FUNCTOR_NOT_SET};
  static constexpr int shot_i_rig_instance_index_ = 0;
  static constexpr int shot_j_rig_instance_index_ = 1;
};

struct CommonPositionError {
  CommonPositionError(double margin, double std_deviation)
      : margin_(margin), scale_(1.0 / std_deviation) {}

  template <typename T>
  bool operator()(T const* const* p, T* r) const {
    Vec3<T> t1 = ShotPositionFunctor(shot_i_rig_instance_index_,
                                     shot_i_rig_camera_index_)(p);
    Vec3<T> t2 = ShotPositionFunctor(shot_j_rig_instance_index_,
                                     shot_j_rig_camera_index_)(p);
    Eigen::Map<Eigen::Matrix<T, 3, 1> > residual(r);

    // error is : shot_origin_1 - shot_origin_2
    Eigen::Matrix<T, 3, 1> error = t1 - t2;

    // restrict XYerror to some positive margin (?)
    for (int i = 0; i < 2; ++i) {
      error(i) = std::max(T(0.0), ceres::abs(error(i)) - T(margin_));
    }
    residual = T(scale_) * error;
    return true;
  }

  double margin_;
  double scale_;

  int shot_i_rig_camera_index_{FUNCTOR_NOT_SET};
  int shot_j_rig_camera_index_{FUNCTOR_NOT_SET};
  static constexpr int shot_i_rig_instance_index_ = 0;
  static constexpr int shot_j_rig_instance_index_ = 1;
};
}  // namespace bundle
