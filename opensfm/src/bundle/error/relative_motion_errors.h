#pragma once

#include <bundle/data/pose.h>
#include <bundle/error/error_utils.h>

#include <Eigen/Eigen>

namespace bundle {

struct RelativeMotionError {
  RelativeMotionError(const Eigen::VectorXd& Rtij,
                      const Eigen::MatrixXd& scale_matrix)
      : Rtij_(Rtij), scale_matrix_(scale_matrix) {}

  template <typename T>
  Eigen::Matrix<T, 6, 1> Error(const T* const shot_i, const T* const scale,
                               const T* const shot_j) const {
    // Get rotation and translation values.
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > Ri(shot_i + Pose::Parameter::RX);
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > Rj(shot_j + Pose::Parameter::RX);
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > ti(shot_i + Pose::Parameter::TX);
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > tj(shot_j + Pose::Parameter::TX);
    Eigen::Matrix<T, 6, 1> residual;

    // Compute rotation residual: log( Rij Ri Rj^t )  ->  log( Rij Ri^t Rj)
    const Eigen::Matrix<T, 3, 1> Rij =
        Rtij_.segment<3>(Pose::Parameter::RX).cast<T>();
    residual.segment(0, 3) = MultRotations(Rij, (-Ri).eval(), Rj.eval());

    // Compute translation residual: tij - scale * ( tj - Rj Ri^t ti )  ->  tij
    // - scale * Rj^t * (ti - tj)
    const auto tij = Rtij_.segment<3>(Pose::Parameter::TX).cast<T>();
    residual.segment(3, 3) =
        tij - scale[0] * RotatePoint((-Rj).eval(), (ti - tj).eval());
    return residual;
  }

  template <typename T>
  bool operator()(const T* const shot_i, const T* const scale,
                  const T* const shot_j, T* r) const {
    Eigen::Map<Eigen::Matrix<T, 6, 1> > residual(r);
    residual = scale_matrix_.cast<T>() * Error(shot_i, scale, shot_j);
    return true;
  }

  Eigen::VectorXd Rtij_;
  Eigen::MatrixXd scale_matrix_;
};

struct RelativeSimilarityError : public RelativeMotionError {
  RelativeSimilarityError(const Eigen::VectorXd& Rtij, double Sij,
                          const Eigen::MatrixXd& scale_matrix)
      : RelativeMotionError(Rtij, scale_matrix), Sij_(Sij) {}

  template <typename T>
  bool operator()(const T* const shot_i, const T* const scale_i,
                  const T* const shot_j, const T* const scale_j, T* r) const {
    Eigen::Map<Eigen::Matrix<T, 7, 1> > residual(r);
    residual.segment(0, 6) =
        RelativeMotionError::Error(shot_i, scale_j, shot_j);
    if (scale_i[0] == T(0.0)) {
      return false;
    }
    residual(6) = (T(Sij_) - scale_j[0] / scale_i[0]);
    residual = scale_matrix_.cast<T>() * residual;
    return true;
  }

  double Sij_;
};

struct RelativeRotationError {
  RelativeRotationError(const Vec3d& Rij, const Eigen::Matrix3d& scale_matrix)
      : Rij_(Rij), scale_matrix_(scale_matrix) {}

  template <typename T>
  bool operator()(const T* const shot_i, const T* const shot_j, T* r) const {
    // Get rotation and translation values.
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > Ri(shot_i + Pose::Parameter::RX);
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > Rj(shot_j + Pose::Parameter::RX);
    Eigen::Map<Eigen::Matrix<T, 3, 1> > residual(r);

    // Compute rotation residual: log( Rij Ri Rj^t ) -> log( Rij Ri^t Rj)
    const Eigen::Matrix<T, 3, 1> Rij = Rij_.cast<T>();
    residual =
        scale_matrix_.cast<T>() * MultRotations(Rij, (-Ri).eval(), Rj.eval());
    return true;
  }

  Vec3d Rij_;
  Eigen::Matrix3d scale_matrix_;
};

struct CommonPositionError {
  CommonPositionError(double margin, double std_deviation)
      : margin_(margin), scale_(1.0 / std_deviation) {}

  template <typename T>
  bool operator()(const T* const shot_1, const T* const shot_2, T* r) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > R1(shot_1 + Pose::Parameter::RX);
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > t1(shot_1 + Pose::Parameter::TX);
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > R2(shot_2 + Pose::Parameter::RX);
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > t2(shot_2 + Pose::Parameter::TX);
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
};
}  // namespace bundle
