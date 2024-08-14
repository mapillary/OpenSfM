#pragma once

#include <bundle/bundle_adjuster.h>
#include <bundle/error/error_utils.h>
#include <ceres/sized_cost_function.h>
#include <foundation/types.h>
#include <geometry/functions.h>

#include <unordered_set>

#include "foundation/optional.h"
#include "geometry/camera_instances.h"

namespace bundle {
class ReprojectionError {
 public:
  ReprojectionError(const geometry::ProjectionType& type, const Vec2d& observed,
                    double std_deviation, bool use_rig_camera)
      : type_(type),
        observed_(observed),
        scale_(1.0 / std_deviation),
        use_rig_camera_(use_rig_camera) {}

 protected:
  const geometry::ProjectionType type_;
  const Vec2d observed_;
  const double scale_;
  const bool use_rig_camera_;
};

class ReprojectionError2D : public ReprojectionError {
 public:
  using ReprojectionError::ReprojectionError;
  constexpr static int Size = 2;

  template <typename T>
  bool operator()(const T* const camera, const T* const rig_instance,
                  const T* const rig_camera, const T* const point,
                  T* residuals) const {
    T scale_one = T(1.0);
    T camera_point[3];
    const T* const actual_rig_camera = use_rig_camera_ ? rig_camera : nullptr;
    WorldToCameraCoordinatesRig(&scale_one, rig_instance, actual_rig_camera,
                                point, &camera_point[0]);

    // Apply camera projection
    T predicted[2];
    geometry::Dispatch<geometry::ProjectFunction>(type_, camera_point, camera,
                                                  predicted);

    // The error is the difference between the predicted and observed position
    residuals[0] = T(scale_) * (predicted[0] - T(observed_[0]));
    residuals[1] = T(scale_) * (predicted[1] - T(observed_[1]));

    return true;
  }
};

template <int C>
class ReprojectionError2DAnalytic
    : public ReprojectionError,
      public ceres::SizedCostFunction<2, C, 6, 6, 3> {
 public:
  using ReprojectionError::ReprojectionError;
  constexpr static int Size = 2;

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const {
    const double* camera = parameters[0];
    const double* rig_instance = parameters[1];
    const double* rig_camera = parameters[2];
    const double* point = parameters[3];

    constexpr int PointSize = 3;
    double transformed[PointSize];
    double predicted[Size];

    /* Error only */
    if (!jacobians) {
      geometry::PoseFunctor::Forward(point, rig_instance, &transformed[0]);
      if (use_rig_camera_) {
        geometry::PoseFunctor::Forward(&transformed[0], rig_camera,
                                       &transformed[0]);
      }
      geometry::Dispatch<geometry::ProjectFunction>(type_, transformed, camera,
                                                    predicted);
    } /* Jacobian + Error */
    else {
      constexpr int CameraSize = C;
      constexpr int PoseSize = 6;

      // Due to many compile-time definitions for matrix for matrix sizes
      // and strides, we had no choices but to duplicate the rig-camera case
      // and no-rig-camera cases
      if (use_rig_camera_) {
        double all_params[PoseSize + PoseSize + CameraSize];
        for (int i = 0; i < PoseSize; ++i) {
          all_params[i] = rig_instance[i];
        }
        for (int i = 0; i < PoseSize; ++i) {
          all_params[PoseSize + i] = rig_camera[i];
        }
        for (int i = 0; i < CameraSize; ++i) {
          all_params[2 * PoseSize + i] = camera[i];
        }

        constexpr int StrideFull = PointSize + CameraSize + 2 * PoseSize;
        double jacobian[Size * StrideFull];
        geometry::Dispatch<geometry::ProjectRigPoseDerivatives>(
            type_, point, &all_params[0], &predicted[0], &jacobian[0]);

        // Unfold big jacobian stored as | point | jac_rig_instance |
        // jac_rig_camera | camera | per block We also take the opportunity to
        // apply the scale
        double *jac_camera = jacobians[0], *jac_rig_instance = jacobians[1],
               *jac_rig_camera = jacobians[2], *jac_point = jacobians[3];
        Eigen::Map<Eigen::Matrix<double, Size, StrideFull, Eigen::RowMajor>>
            map_jac_big(jacobian);

        if (jac_point) {
          Eigen::Map<Eigen::Matrix<double, Size, PointSize, Eigen::RowMajor>>
              map_jac_point(jac_point);
          map_jac_point =
              scale_ * map_jac_big.template block<Size, PointSize>(0, 0);
        }
        if (jac_rig_instance) {
          Eigen::Map<Eigen::Matrix<double, Size, PoseSize, Eigen::RowMajor>>
              map_jac_rig_instance(jac_rig_instance);
          map_jac_rig_instance =
              scale_ * map_jac_big.template block<Size, PoseSize>(0, PointSize);
        }
        if (jac_rig_camera) {
          Eigen::Map<Eigen::Matrix<double, Size, PoseSize, Eigen::RowMajor>>
              map_jac_rig_camera(jac_rig_camera);
          map_jac_rig_camera =
              scale_ * map_jac_big.template block<Size, PoseSize>(
                           0, PointSize + PoseSize);
        }
        if (jac_camera) {
          // Eigen doesn't like vectors as 1-dim matrices (CameraSize == 1),
          // using simple assignments instead of Eigen's Map
          for (int i = 0; i < Size; ++i) {
            for (int j = 0; j < CameraSize; ++j) {
              jac_camera[i * CameraSize + j] =
                  scale_ *
                  jacobian[i * StrideFull + PointSize + 2 * PoseSize + j];
            }
          }
        }
      } else {
        double all_params[PoseSize + CameraSize];
        for (int i = 0; i < PoseSize; ++i) {
          all_params[i] = rig_instance[i];
        }
        for (int i = 0; i < CameraSize; ++i) {
          all_params[PoseSize + i] = camera[i];
        }

        constexpr int StrideFull = PointSize + CameraSize + PoseSize;
        double jacobian[Size * StrideFull];
        geometry::Dispatch<geometry::ProjectPoseDerivatives>(
            type_, point, &all_params[0], &predicted[0], &jacobian[0]);

        // Unfold big jacobian stored as | point | jac_rig_instance |
        // jac_rig_camera | camera | per block We also take the opportunity to
        // apply the scale
        double *jac_camera = jacobians[0], *jac_rig_instance = jacobians[1],
               *jac_rig_camera = jacobians[2], *jac_point = jacobians[3];
        Eigen::Map<Eigen::Matrix<double, Size, StrideFull, Eigen::RowMajor>>
            map_jac_big(jacobian);

        if (jac_point) {
          Eigen::Map<Eigen::Matrix<double, Size, PointSize, Eigen::RowMajor>>
              map_jac_point(jac_point);
          map_jac_point =
              scale_ * map_jac_big.template block<Size, PointSize>(0, 0);
        }
        if (jac_rig_instance) {
          Eigen::Map<Eigen::Matrix<double, Size, PoseSize, Eigen::RowMajor>>
              map_jac_rig_instance(jac_rig_instance);
          map_jac_rig_instance =
              scale_ * map_jac_big.template block<Size, PoseSize>(0, PointSize);
        }
        if (jac_rig_camera) {
          Eigen::Map<Eigen::Matrix<double, Size, PoseSize, Eigen::RowMajor>>
              map_jac_rig_camera(jac_rig_camera);
          map_jac_rig_camera.setZero();
        }
        if (jac_camera) {
          // Eigen doesn't like vectors as 1-dim matrices (CameraSize == 1),
          // using simple assignments instead of Eigen's Map
          for (int i = 0; i < Size; ++i) {
            for (int j = 0; j < CameraSize; ++j) {
              jac_camera[i * CameraSize + j] =
                  scale_ * jacobian[i * StrideFull + PointSize + PoseSize + j];
            }
          }
        }
      }
    }

    // The error is the difference between the predicted and observed position
    for (int i = 0; i < Size; ++i) {
      residuals[i] = scale_ * (predicted[i] - observed_[i]);
    }
    return true;
  }
};

class ReprojectionError3D : public ReprojectionError {
 public:
  constexpr static int Size = 3;

  ReprojectionError3D(const geometry::ProjectionType& type,
                      const Vec2d& observed, double std_deviation,
                      bool use_rig_camera)
      : ReprojectionError(type, observed, std_deviation, use_rig_camera) {
    const double lon = observed[0] * 2 * M_PI;
    const double lat = -observed[1] * 2 * M_PI;
    bearing_vector_[0] = std::cos(lat) * std::sin(lon);
    bearing_vector_[1] = -std::sin(lat);
    bearing_vector_[2] = std::cos(lat) * std::cos(lon);
  }

  template <typename T>
  bool operator()(const T* const, const T* const rig_instance,
                  const T* const rig_camera, const T* const point,
                  T* residuals) const {
    T scale_one = T(1.0);
    Vec3<T> predicted;
    const T* const actual_rig_camera = use_rig_camera_ ? rig_camera : nullptr;
    WorldToCameraCoordinatesRig(&scale_one, rig_instance, actual_rig_camera,
                                point, predicted.data());
    predicted.normalize();

    /* Difference between projected vector and observed bearing vector. We use
     * the difference between unit vectors as an approximation to the angle for
     * small angles. */
    Eigen::Map<Vec3<T>> residuals_mapped(residuals);
    residuals_mapped = T(scale_) * (predicted - bearing_vector_.cast<T>());
    return true;
  }

 protected:
  Vec3d bearing_vector_;
};

class ReprojectionError3DAnalytic
    : protected ReprojectionError3D,
      public ceres::SizedCostFunction<3, 1, 6, 6, 3> {
 public:
  constexpr static int Size = 3;
  using ReprojectionError3D::ReprojectionError3D;

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const {
    const double* rig_instance = parameters[1];
    const double* rig_camera = parameters[2];
    const double* point = parameters[3];

    Vec3d transformed;
    /* Error only */
    if (!jacobians) {
      geometry::PoseFunctor::Forward(point, rig_instance, transformed.data());
      if (use_rig_camera_) {
        geometry::PoseFunctor::Forward(transformed.data(), rig_camera,
                                       transformed.data());
      }
      transformed.normalize();
    } /* Jacobian + Error */
    else {
      constexpr int PointSize = 3;
      constexpr int PoseSize = 6;

      // Due to many compile-time definitions for matrix for matrix sizes
      // and strides, we had no choices but to duplicate the rig-camera case
      // and no-rig-camera cases
      if (use_rig_camera_) {
        double all_params[PoseSize + PoseSize];
        for (int i = 0; i < PoseSize; ++i) {
          all_params[i] = rig_instance[i];
        }
        for (int i = 0; i < PoseSize; ++i) {
          all_params[PoseSize + i] = rig_camera[i];
        }

        constexpr int StrideFull = PoseSize + PoseSize + PointSize;
        double jacobian[Size * StrideFull];
        geometry::Dispatch<geometry::RigPoseNormalizedDerivatives>(
            geometry::ProjectionType::SPHERICAL, point, &all_params[0],
            transformed.data(), &jacobian[0]);

        // Unfold big jacobian stored as | point | rig instance pose | rig
        // camera pose per block, We also take the opportunity to apply the
        // scale
        double *jac_camera = jacobians[0], *jac_rig_instance = jacobians[1],
               *jac_rig_camera = jacobians[2], *jac_point = jacobians[3];
        Eigen::Map<Eigen::Matrix<double, Size, StrideFull, Eigen::RowMajor>>
            map_jac_big(jacobian);

        if (jac_camera) {
          for (int i = 0; i < Size; ++i) {
            jac_camera[i] = 0.;
          }
        }
        if (jac_point) {
          Eigen::Map<Eigen::Matrix<double, Size, PointSize, Eigen::RowMajor>>
              map_jac_point(jac_point);
          map_jac_point =
              scale_ * map_jac_big.template block<Size, PointSize>(0, 0);
        }
        if (jac_rig_instance) {
          Eigen::Map<Eigen::Matrix<double, Size, PoseSize, Eigen::RowMajor>>
              map_jac_rig_instance(jac_rig_instance);
          map_jac_rig_instance =
              scale_ * map_jac_big.template block<Size, PoseSize>(0, PointSize);
        }
        if (jac_rig_camera) {
          Eigen::Map<Eigen::Matrix<double, Size, PoseSize, Eigen::RowMajor>>
              map_jac_rig_camera(jac_rig_camera);
          map_jac_rig_camera =
              scale_ * map_jac_big.template block<Size, PoseSize>(
                           0, PointSize + PoseSize);
        }
      } else {
        double all_params[PoseSize];
        for (int i = 0; i < PoseSize; ++i) {
          all_params[i] = rig_instance[i];
        }

        constexpr int StrideFull = PoseSize + PointSize;
        double jacobian[Size * StrideFull];
        geometry::Dispatch<geometry::PoseNormalizedDerivatives>(
            geometry::ProjectionType::SPHERICAL, point, &all_params[0],
            transformed.data(), &jacobian[0]);

        // Unfold big jacobian stored as | point | rig instance pose
        // per block, We also take the opportunity to apply the scale
        double *jac_camera = jacobians[0], *jac_rig_instance = jacobians[1],
               *jac_rig_camera = jacobians[2], *jac_point = jacobians[3];
        Eigen::Map<Eigen::Matrix<double, Size, StrideFull, Eigen::RowMajor>>
            map_jac_big(jacobian);

        if (jac_camera) {
          for (int i = 0; i < Size; ++i) {
            jac_camera[i] = 0.;
          }
        }
        if (jac_point) {
          Eigen::Map<Eigen::Matrix<double, Size, PointSize, Eigen::RowMajor>>
              map_jac_point(jac_point);
          map_jac_point =
              scale_ * map_jac_big.template block<Size, PointSize>(0, 0);
        }
        if (jac_rig_instance) {
          Eigen::Map<Eigen::Matrix<double, Size, PoseSize, Eigen::RowMajor>>
              map_jac_rig_instance(jac_rig_instance);
          map_jac_rig_instance =
              scale_ * map_jac_big.template block<Size, PoseSize>(0, PointSize);
        }
        if (jac_rig_camera) {
          Eigen::Map<Eigen::Matrix<double, Size, PoseSize, Eigen::RowMajor>>
              map_jac_rig_camera(jac_rig_camera);
          map_jac_rig_camera.setZero();
        }
      }
    }

    // The error is the difference between the predicted and observed position
    for (int i = 0; i < Size; ++i) {
      residuals[i] = scale_ * (transformed[i] - bearing_vector_[i]);
    }
    return true;
  }
};
}  // namespace bundle
