#pragma once

#include <bundle/data/pose.h>
#include <bundle/error/error_utils.h>
#include <bundle/error/position_functors.h>

#include <Eigen/Eigen>

namespace bundle {

struct RelativeMotionError {
  RelativeMotionError(const Eigen::VectorXd& Rtij,
                      const Eigen::MatrixXd& scale_matrix)
      : Rtij_(Rtij),
        scale_matrix_(scale_matrix),
        shot_i_rig_camera_index_(FUNCTOR_NOT_SET),
        shot_j_rig_camera_index_(FUNCTOR_NOT_SET) {}

  template <typename T>
  Eigen::Matrix<T, 6, 1> Error(T const* const* p) const {
    // Get rotation and translation values.
    Vec3<T> Ri = ShotRotationFunctor(shot_i_rig_instance_index_,
                                     shot_i_rig_camera_index_)(p);
    Vec3<T> ti = ShotPositionFunctor(shot_i_rig_instance_index_,
                                     shot_i_rig_camera_index_)(p);
    Vec3<T> Rj = ShotRotationFunctor(shot_j_rig_instance_index_,
                                     shot_j_rig_camera_index_)(p);
    Vec3<T> tj = ShotPositionFunctor(shot_j_rig_instance_index_,
                                     shot_j_rig_camera_index_)(p);
    Eigen::Matrix<T, 6, 1> residual;

    // Compute rotation residual: log( Rij Ri Rj^t )  ->  log( Rij Ri^t Rj)
    const Eigen::Matrix<T, 3, 1> Rij =
        Rtij_.segment<3>(Pose::Parameter::RX).cast<T>();
    residual.segment(0, 3) = MultRotations(Rij, (-Ri).eval(), Rj.eval());

    // Compute translation residual: tij - scale * ( tj - Rj Ri^t ti )  ->  tij
    // - scale * Rj^t * (ti - tj)
    const auto scale = p[scale_index_];
    const auto tij = Rtij_.segment<3>(Pose::Parameter::TX).cast<T>();
    residual.segment(3, 3) =
        tij - scale[0] * RotatePoint((-Rj).eval(), (ti - tj).eval());
    return residual;
  }

  template <typename T>
  bool operator()(T const* const* p, T* r) const {
    Eigen::Map<Eigen::Matrix<T, 6, 1> > residual(r);
    residual = scale_matrix_.cast<T>() * Error(p);
    return true;
  }

  Eigen::VectorXd Rtij_;
  Eigen::MatrixXd scale_matrix_;
  int shot_i_rig_camera_index_{FUNCTOR_NOT_SET};
  int shot_j_rig_camera_index_{FUNCTOR_NOT_SET};
  static constexpr int shot_i_rig_instance_index_ = 0;
  static constexpr int shot_j_rig_instance_index_ = 1;
  static constexpr int scale_index_ = 2;
};

struct RelativeSimilarityError : public RelativeMotionError {
  RelativeSimilarityError(const Eigen::VectorXd& Rtij, double Sij,
                          const Eigen::MatrixXd& scale_matrix)
      : RelativeMotionError(Rtij, scale_matrix), Sij_(Sij) {}

  template <typename T>
  bool operator()(T const* const* p, T* r) const {
    auto scale_i = p[scale_i_index_];
    auto scale_j = p[scale_j_index_];

    Eigen::Map<Eigen::Matrix<T, 7, 1> > residual(r);
    residual.segment(0, 6) = RelativeMotionError::Error(p);
    if (scale_i[0] == T(0.0)) {
      return false;
    }
    residual(6) = (T(Sij_) - scale_j[0] / scale_i[0]);
    residual = scale_matrix_.cast<T>() * residual;
    return true;
  }

  double Sij_;
  static constexpr int scale_i_index_ = 2;
  static constexpr int scale_j_index_ = 3;
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
