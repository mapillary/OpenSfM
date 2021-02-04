#pragma once
#include <geometry/camera.h>
#include <geometry/pose.h>
#include <map/defines.h>
#include <map/landmark.h>
#include <sfm/observation.h>

#include <Eigen/Eigen>
#include <iostream>
#include <unordered_map>

namespace map {
class Map;

struct ShotMesh {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void SetVertices(const MatXd& vertices) { vertices_ = vertices; }
  void SetFaces(const MatXd& faces) { faces_ = faces; }
  MatXd GetFaces() const { return faces_; }
  MatXd GetVertices() const { return vertices_; }
  MatXd vertices_;
  MatXd faces_;
};

template <typename T>
class ShotMeasurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool HasValue() const { return has_value_; }
  T Value() const { return value_; }
  void SetValue(const T& v) {
    value_ = v;
    has_value_ = true;
  }
  void Reset() { has_value_ = false; }

 private:
  bool has_value_{false};
  T value_;
};

struct ShotMeasurements {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ShotMeasurement<double> capture_time_;
  ShotMeasurement<Vec3d> gps_position_;
  ShotMeasurement<double> gps_accuracy_;
  ShotMeasurement<double> compass_accuracy_;
  ShotMeasurement<double> compass_angle_;
  ShotMeasurement<Vec3d> accelerometer_;
  ShotMeasurement<int> orientation_;
  ShotMeasurement<std::string> sequence_key_;
  void Set(const ShotMeasurements& other);
};

class Shot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Shot(const ShotId& shot_id, const Camera* const shot_camera,
       const geometry::Pose& pose);
  // Workaround for pickle that makes it possible for the shot to have camera
  // outside of the reconstruction.
  Shot(const ShotId& shot_id, std::unique_ptr<Camera> cam,
       const geometry::Pose& pose);

  ShotId GetId() const { return id_; }

  // read-only access
  size_t ComputeNumValidLandmarks(const int min_obs_thr = 1) const;

  const std::map<
      Landmark*, Observation, KeyCompare,
      Eigen::aligned_allocator<std::pair<Landmark* const, Observation>>>&
  GetLandmarkObservations() const {
    return landmark_observations_;
  }
  std::map<Landmark*, Observation, KeyCompare,
           Eigen::aligned_allocator<std::pair<Landmark* const, Observation>>>&
  GetLandmarkObservations() {
    return landmark_observations_;
  }

  const Observation& GetObservation(const FeatureId id) const {
    return landmark_observations_.at(landmark_id_.at(id));
  }

  std::vector<Landmark*> ComputeValidLandmarks() {
    std::vector<Landmark*> valid_landmarks;
    valid_landmarks.reserve(landmark_observations_.size());
    for (const auto& lm_obs : landmark_observations_) {
      valid_landmarks.push_back(lm_obs.first);
    }
    return valid_landmarks;
  }
  void RemoveLandmarkObservation(const FeatureId id);

  void CreateObservation(Landmark* lm, const Observation& obs) {
    landmark_observations_.insert(std::make_pair(lm, obs));
    landmark_id_.insert(std::make_pair(obs.feature_id, lm));
  }

  Observation* GetLandmarkObservation(Landmark* lm) {
    return &landmark_observations_.at(lm);
  }

  ShotMeasurements& GetShotMeasurements() { return shot_measurements_; }
  const ShotMeasurements& GetShotMeasurements() const { return shot_measurements_; }
  void SetShotMeasurements(const ShotMeasurements& other) {
    shot_measurements_.Set(other);
  }

  void SetPose(const geometry::Pose& pose) { pose_ = pose; }
  const geometry::Pose& GetPose() const { return pose_; }
  geometry::Pose& GetPose() { return pose_; }
  Mat4d GetWorldToCam() const { return pose_.WorldToCamera(); }
  Mat4d GetCamToWorld() const { return pose_.CameraToWorld(); }

  void ScalePose(const double scale);
  void ScaleLandmarks(const double scale);
  // Comparisons
  bool operator==(const Shot& shot) const { return id_ == shot.id_; }
  bool operator!=(const Shot& shot) const { return !(*this == shot); }
  bool operator<(const Shot& shot) const { return id_ < shot.id_; }
  bool operator<=(const Shot& shot) const { return id_ <= shot.id_; }
  bool operator>(const Shot& shot) const { return id_ > shot.id_; }
  bool operator>=(const Shot& shot) const { return id_ >= shot.id_; }
  std::string GetCameraName() const { return shot_camera_->id; }
  const Camera* const GetCamera() const { return shot_camera_; }

  Vec2d Project(const Vec3d& global_pos) const;
  MatX2d ProjectMany(const MatX3d& points) const;

  Vec3d Bearing(const Vec2d& point) const;
  MatX3d BearingMany(const MatX2d& points) const;

  MatXd GetCovariance() const { return covariance; };
  void SetCovariance(const MatXd& cov) { covariance = cov; };

 public:
  const ShotId id_;  // the file name
  const Camera* const shot_camera_;
  ShotUniqueId unique_id_;

  ShotMeasurements shot_measurements_;  // metadata
  ShotMesh mesh;
  MatXd covariance;
  long int merge_cc;
  double scale;

 private:
  geometry::Pose pose_;
  // In OpenSfM, we use a map to reproduce a similar behaviour
  std::map<Landmark*, Observation, KeyCompare,
           Eigen::aligned_allocator<std::pair<Landmark* const, Observation>>>
      landmark_observations_;
  std::unordered_map<FeatureId, Landmark*> landmark_id_;
  // Workaround for pickle that makes it possible for the shot to have camera
  // outside of the reconstruction.
  std::unique_ptr<Camera> own_camera_;
};
}  // namespace map
