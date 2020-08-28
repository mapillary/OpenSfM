#pragma once

#include <ceres/sized_cost_function.h>
#include <foundation/types.h>
#include <geometry/camera_functions.h>
#include <bundle/bundle_adjuster.h>

template <class PointFunc>
struct BABearingError {
  BABearingError(const Eigen::Vector3d& bearing,
                 double bearing_std_deviation,
                 const PointFunc& pos_func)
      : bearing_(bearing)
      , bearing_scale_(1.0 / bearing_std_deviation)
      , pos_func_(pos_func)
  {}

  template <typename T>
  bool operator()(T const* const* p, T* r) const {
    Eigen::Map< Eigen::Matrix<T,3,1> > residual(r);
    const auto shot_point = pos_func_(p);
    residual = (bearing_.normalized().template cast<T>() - shot_point.normalized()) * T(bearing_scale_);
    return true;
  }

  Eigen::Vector3d bearing_;
  double bearing_scale_;
  PointFunc pos_func_;
};

template <typename T>
void WorldToCameraCoordinates(const T* const shot,
                              const T world_point[3],
                              T camera_point[3]) {
  const T pt[3] = {
    world_point[0] - shot[BA_SHOT_TX],
    world_point[1] - shot[BA_SHOT_TY],
    world_point[2] - shot[BA_SHOT_TZ]
  };
  const T Rt[3] = {
    -shot[BA_SHOT_RX],
    -shot[BA_SHOT_RY],
    -shot[BA_SHOT_RZ]
  };
  ceres::AngleAxisRotatePoint(Rt, pt, camera_point);
}

template <class DATA>
struct BADataPriorError {
 public:
  // Parameter scaling
  enum class ScaleType { LINEAR = 0, LOGARITHMIC = 1 };

  BADataPriorError(BAData<DATA>* ba_data)
      : ba_data_(ba_data), count_(ba_data->GetValueData().rows()) {
    const auto sigma = ba_data->GetSigmaData();
    scales_.resize(sigma.rows(), 1);
    for (int i = 0; i < count_; ++i) {
      scales_(i) = 1.0 / std::max(sigma(i), std::numeric_limits<double>::epsilon());
    }
  }

  void SetScaleType(int index, const ScaleType& type) {
    if(index > count_){
      throw std::runtime_error("Parameter index out-of-range");
    }
    scale_types_[index] = type;
  }

  template <typename T>
  bool operator()(const T* const parameters, T* residuals) const {
    Eigen::Map<VecX<T>> residuals_mapped(residuals, count_);
    Eigen::Map<const VecX<T>> parameters_values(parameters, count_);
    const VecX<T> prior_values = ba_data_->GetPriorData().template cast<T>();
    for (int i = 0; i < count_; ++i) {
      auto scale_type = ScaleType::LINEAR;
      const auto scale_type_find = scale_types_.find(i);
      if (scale_type_find != scale_types_.end()) {
        scale_type = scale_type_find->second;
      }

      T error = T(0.);
      switch (scale_type) {
        case ScaleType::LINEAR:
          error = parameters_values(i) - prior_values(i);
          break;
        case ScaleType::LOGARITHMIC:
          error = log(parameters_values(i) / prior_values(i));
          break;
      }
      residuals_mapped(i) = scales_(i) * error;
    }
    return true;
  }

 private:
  int count_;
  VecXd scales_;
  std::unordered_map<int, ScaleType>
      scale_types_;  // Per-parameter prior scaling (default is linear)
  BAData<DATA>* ba_data_;
};

class ReprojectionError{
public:
  ReprojectionError(const ProjectionType& type, const Vec2d& observed, double std_deviation)
      : type_(type), observed_(observed), scale_(1.0 / std_deviation) {}

protected:
  ProjectionType type_;
  Vec2d observed_;
  double scale_;
};

class ReprojectionError2D : public ReprojectionError{
  public:
   using ReprojectionError::ReprojectionError;
   constexpr static int Size = 2;

  template <typename T>
  bool operator()(const T* const camera, const T* const shot,
                  const T* const point, T* residuals) const {
     T camera_point[3];
     WorldToCameraCoordinates(shot, point, camera_point);

     // Apply camera projection
     T predicted[2];
     Dispatch<ProjectFunction>(type_, camera_point, camera, predicted);

     // The error is the difference between the predicted and observed position
     for (int i = 0; i < 2; ++i) {
       residuals[i] = T(scale_) * (predicted[i] - T(observed_[i]));
     }

     return true;
  }
};

template<int C>
class ReprojectionError2DAnalytic : public ReprojectionError,
                                    public ceres::SizedCostFunction<2, C, 6, 3> {
 public:
  using ReprojectionError::ReprojectionError;
  constexpr static int Size = 2;

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const {
    const double* camera = parameters[0];
    const double* shot = parameters[1];
    const double* point = parameters[2];

    constexpr int PointSize = 3;
    double transformed[PointSize];
    double predicted[Size];

    /* Error only */
    if (!jacobians) {
      Pose::Forward(point, shot, &transformed[0]);
      Dispatch<ProjectFunction>(type_, transformed, camera, predicted);
    } /* Jacobian + Error */
    else {
      constexpr int CameraSize = C;
      constexpr int PoseSize = 6;

      double all_params[PoseSize + CameraSize];
      for (int i = 0; i < PoseSize; ++i) {
        all_params[i] = shot[i];
      }
      for (int i = 0; i < CameraSize; ++i) {
        all_params[PoseSize + i] = camera[i];
      }

      constexpr int StrideFull = PointSize + CameraSize + PoseSize;
      double jacobian[Size * StrideFull];
      Dispatch<ProjectPoseDerivatives>(type_, point, &all_params[0], &predicted[0], &jacobian[0]);

      // Unfold big jacobian stored as | point | pose | camera | per block
      // We also take the opportunity to apply the scale
      double* jac_camera = jacobians[0], * jac_pose = jacobians[1], * jac_point = jacobians[2];
      if (jac_point) {
        for (int i = 0; i < Size; ++i) {
          for (int j = 0; j < PointSize; ++j) {
            jac_point[i * PointSize + j] =
                scale_ * jacobian[i * StrideFull + j];
          }
        }
      }
      if (jac_pose) {
        for (int i = 0; i < Size; ++i) {
          for (int j = 0; j < PoseSize; ++j) {
            jac_pose[i * PoseSize + j] =
                scale_ * jacobian[i * StrideFull + PointSize + j];
          }
        }
      }
      if (jac_camera) {
        for (int i = 0; i < Size; ++i) {
          for (int j = 0; j < CameraSize; ++j) {
            jac_camera[i * CameraSize + j] =
                scale_ * jacobian[i * StrideFull + PointSize + PoseSize + j];
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

  ReprojectionError3D(const ProjectionType& type, const Vec2d& observed,
                      double std_deviation)
      : ReprojectionError(type, observed, std_deviation) {
    double lon = observed[0] * 2 * M_PI;
    double lat = -observed[1] * 2 * M_PI;
    bearing_vector_[0] = std::cos(lat) * std::sin(lon);
    bearing_vector_[1] = -std::sin(lat);
    bearing_vector_[2] = std::cos(lat) * std::cos(lon);
  }

  template <typename T>
  bool operator()(const T* const camera, const T* const shot,
                  const T* const point, T* residuals) const {
    Vec3<T> predicted;
    WorldToCameraCoordinates(shot, point, predicted.data());
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
      public ceres::SizedCostFunction<3, 1, 6, 3> {
 public:
  constexpr static int Size = 3;
  using ReprojectionError3D::ReprojectionError3D;

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const {
    const double* shot = parameters[1];
    const double* point = parameters[2];

    Vec3d transformed;
    /* Error only */
    if (!jacobians) {
      Pose::Forward(point, shot, transformed.data());
      transformed.normalize();
    } /* Jacobian + Error */
    else {
      constexpr int PointSize = 3;
      constexpr int PoseSize = 6;
      constexpr int StrideFull = PoseSize + PointSize;
      double jacobian[Size * StrideFull];
      Dispatch<PoseNormalizedDerivatives>(type_, point, shot,
                                          transformed.data(), &jacobian[0]);

      // Unfold big jacobian stored as | point | pose | per block
      // We also take the opportunity to apply the scale
      double* jac_camera = jacobians[0], * jac_pose = jacobians[1], * jac_point = jacobians[2];
      if (jac_camera) {
        for (int i = 0; i < Size; ++i) {
          jac_camera[i] = 0.;
        }
      }
      if (jac_point) {
        for (int i = 0; i < Size; ++i) {
          for (int j = 0; j < PointSize; ++j) {
            jac_point[i * PointSize + j] =
                scale_ * jacobian[i * StrideFull + j];
          }
        }
      }
      if (jac_pose) {
        for (int i = 0; i < Size; ++i) {
          for (int j = 0; j < PoseSize; ++j) {
            jac_pose[i * PoseSize + j] =
                scale_ * jacobian[i * StrideFull + PointSize + j];
          }
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
