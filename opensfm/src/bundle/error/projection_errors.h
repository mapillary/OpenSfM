#pragma once

#include <bundle/bundle_adjuster.h>
#include <ceres/sized_cost_function.h>
#include <foundation/types.h>
#include <geometry/functions.h>

#include <unordered_set>

namespace bundle {

template <class PointFunc>
struct BABearingError {
  BABearingError(const Eigen::Vector3d& bearing, double bearing_std_deviation,
                 const PointFunc& pos_func)
      : bearing_(bearing),
        bearing_scale_(1.0 / bearing_std_deviation),
        pos_func_(pos_func) {}

  template <typename T>
  bool operator()(T const* const* p, T* r) const {
    Eigen::Map<Eigen::Matrix<T, 3, 1>> residual(r);
    const auto shot_point = pos_func_(p);
    residual =
        (bearing_.normalized().template cast<T>() - shot_point.normalized()) *
        T(bearing_scale_);
    return true;
  }

  double bearing_scale_;
  Eigen::Vector3d bearing_;
  PointFunc pos_func_;
};

template <typename T>
void WorldToCameraCoordinates(const T* const shot, const T world_point[3],
                              T camera_point[3]) {
  const T pt[3] = {world_point[0] - shot[Pose::Parameter::TX],
                   world_point[1] - shot[Pose::Parameter::TY],
                   world_point[2] - shot[Pose::Parameter::TZ]};
  const T Rt[3] = {-shot[Pose::Parameter::RX], -shot[Pose::Parameter::RY],
                   -shot[Pose::Parameter::RZ]};
  ceres::AngleAxisRotatePoint(Rt, pt, camera_point);
}

template <class DATA>
struct DataPriorError {
 public:
  // Parameter scaling
  enum class ScaleType { LINEAR = 0, LOGARITHMIC = 1 };

  explicit DataPriorError(Data<DATA>* ba_data) : ba_data_(ba_data) {
    const auto sigma = ba_data->GetSigmaData();
    for (int i = 0; i < sigma.rows(); ++i) {
      indexes_.insert(i);
    }

    scales_.resize(sigma.rows(), 1);
    for (const auto& index : indexes_) {
      scales_(index) =
          1.0 / std::max(sigma(index), std::numeric_limits<double>::epsilon());
    }
  }

  void SetScaleType(int index, const ScaleType& type) {
    if (indexes_.find(index) == indexes_.end()) {
      throw std::runtime_error("Parameter index out-of-range");
    }
    scale_types_[index] = type;
  }

  void SetConstrainedDataIndexes(const std::vector<int>& indexes) {
    indexes_.clear();
    indexes_.insert(indexes.begin(), indexes.end());
  }

  template <typename T>
  bool operator()(const T* const parameters, T* residuals) const {
    const auto residual_size = indexes_.size();
    const auto parameter_size = ba_data_->GetSigmaData().rows();

    Eigen::Map<VecX<T>> residuals_mapped(residuals, residual_size);
    Eigen::Map<const VecX<T>> parameters_values(parameters, parameter_size);
    const VecX<T> prior_values = ba_data_->GetPriorData().template cast<T>();

    int residual_index = 0;
    for (const auto& index : indexes_) {
      auto scale_type = ScaleType::LINEAR;
      const auto scale_type_find = scale_types_.find(index);
      if (scale_type_find != scale_types_.end()) {
        scale_type = scale_type_find->second;
      }

      T error = T(0.);
      switch (scale_type) {
        case ScaleType::LINEAR:
          error = parameters_values(index) - prior_values(index);
          break;
        case ScaleType::LOGARITHMIC:
          error = log(parameters_values(index) / prior_values(index));
          break;
      }
      residuals_mapped(residual_index++) = scales_(index) * error;
    }
    return true;
  }

 private:
  Data<DATA>* ba_data_;
  std::set<int> indexes_;
  VecXd scales_;
  std::unordered_map<int, ScaleType>
      scale_types_;  // Per-parameter prior scaling (default is linear)
};

class ReprojectionError {
 public:
  ReprojectionError(const geometry::ProjectionType& type, const Vec2d& observed,
                    double std_deviation)
      : type_(type), observed_(observed), scale_(1.0 / std_deviation) {}

 protected:
  geometry::ProjectionType type_;
  Vec2d observed_;
  double scale_;
};

class ReprojectionError2D : public ReprojectionError {
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
    geometry::Dispatch<geometry::ProjectFunction>(type_, camera_point, camera,
                                                  predicted);

    // The error is the difference between the predicted and observed position
    for (int i = 0; i < 2; ++i) {
      residuals[i] = T(scale_) * (predicted[i] - T(observed_[i]));
    }

    return true;
  }
};

class RigReprojectionError2D : public ReprojectionError {
 public:
  using ReprojectionError::ReprojectionError;
  constexpr static int Size = 2;

  template <typename T>
  bool operator()(const T* const camera, const T* const rig_instance,
                  const T* const rig_camera, const T* const point,
                  T* residuals) const {
    T instance_point[3];
    WorldToCameraCoordinates(rig_instance, point, instance_point);

    T camera_point[3];
    WorldToCameraCoordinates(rig_camera, instance_point, camera_point);

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
      geometry::PoseFunctor::Forward(point, shot, &transformed[0]);
      geometry::Dispatch<geometry::ProjectFunction>(type_, transformed, camera,
                                                    predicted);
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
      geometry::Dispatch<geometry::ProjectPoseDerivatives>(
          type_, point, &all_params[0], &predicted[0], &jacobian[0]);

      // Unfold big jacobian stored as | point | pose | camera | per block
      // We also take the opportunity to apply the scale
      double *jac_camera = jacobians[0], *jac_pose = jacobians[1],
             *jac_point = jacobians[2];

      Eigen::Map<Eigen::Matrix<double, Size, StrideFull, Eigen::RowMajor>>
          map_jac_big(jacobian);

      if (jac_point) {
        Eigen::Map<Eigen::Matrix<double, Size, PointSize, Eigen::RowMajor>>
            map_jac_point(jac_point);
        map_jac_point =
            scale_ * map_jac_big.template block<Size, PointSize>(0, 0);
      }
      if (jac_pose) {
        Eigen::Map<Eigen::Matrix<double, Size, PoseSize, Eigen::RowMajor>>
            map_jac_pose(jac_pose);
        map_jac_pose =
            scale_ * map_jac_big.template block<Size, PoseSize>(0, PointSize);
      }
      if (jac_camera) {
        Eigen::Map<Eigen::Matrix<double, Size, CameraSize, Eigen::RowMajor>>
            map_jac_camera(jac_camera);
        map_jac_camera = scale_ * map_jac_big.template block<Size, CameraSize>(
                                      0, PointSize + PoseSize);
      }
    }

    // The error is the difference between the predicted and observed position
    for (int i = 0; i < Size; ++i) {
      residuals[i] = scale_ * (predicted[i] - observed_[i]);
    }
    return true;
  }
};

template <int C>
class RigReprojectionError2DAnalytic
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
      geometry::PoseFunctor::Forward(&transformed[0], rig_camera,
                                     &transformed[0]);
      geometry::Dispatch<geometry::ProjectFunction>(type_, transformed, camera,
                                                    predicted);
    } /* Jacobian + Error */
    else {
      constexpr int CameraSize = C;
      constexpr int PoseSize = 6;

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
            scale_ *
            map_jac_big.template block<Size, PoseSize>(0, PointSize + PoseSize);
      }
      if (jac_camera) {
        // Eigen doesn't like vectors as 1-dim matrices (CameraSize == 1), using
        // simple assignments instead of Eigen's Map
        for (int i = 0; i < Size; ++i) {
          for (int j = 0; j < CameraSize; ++j) {
            jac_camera[i * CameraSize + j] =
                scale_ *
                jacobian[i * StrideFull + PointSize + 2 * PoseSize + j];
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
                      const Vec2d& observed, double std_deviation)
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
      geometry::PoseFunctor::Forward(point, shot, transformed.data());
      transformed.normalize();
    } /* Jacobian + Error */
    else {
      constexpr int PointSize = 3;
      constexpr int PoseSize = 6;
      constexpr int StrideFull = PoseSize + PointSize;
      double jacobian[Size * StrideFull];
      geometry::Dispatch<geometry::PoseNormalizedDerivatives>(
          type_, point, shot, transformed.data(), &jacobian[0]);

      // Unfold big jacobian stored as | point | pose | per block
      // We also take the opportunity to apply the scale
      double *jac_camera = jacobians[0], *jac_pose = jacobians[1],
             *jac_point = jacobians[2];

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
      if (jac_pose) {
        Eigen::Map<Eigen::Matrix<double, Size, PoseSize, Eigen::RowMajor>>
            map_jac_pose(jac_pose);
        map_jac_pose =
            scale_ * map_jac_big.template block<Size, PoseSize>(0, PointSize);
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
