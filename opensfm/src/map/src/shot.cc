#include <map/landmark.h>
#include <map/rig.h>
#include <map/shot.h>

#include <algorithm>
#include <numeric>
#include <stdexcept>
namespace map {

Shot::Shot(const ShotId& shot_id, const geometry::Camera* const shot_camera,
           const geometry::Pose& pose)
    : id_(shot_id),
      pose_(std::make_unique<geometry::Pose>(pose)),
      shot_camera_(shot_camera) {}

Shot::Shot(const ShotId& shot_id, const geometry::Camera& shot_camera,
           const geometry::Pose& pose)
    : id_(shot_id),
      pose_(std::make_unique<geometry::Pose>(pose)),
      own_camera_(shot_camera),
      shot_camera_(&own_camera_.Value()) {}

void Shot::SetRig(const RigInstance* rig_instance,
                  const RigCamera* rig_camera) {
  rig_instance_.SetValue(rig_instance);
  rig_camera_.SetValue(rig_camera);
  pose_ = std::make_unique<geometry::PoseImmutable>(*pose_);
}

RigInstanceId Shot::GetRigInstanceId() const {
  if (!IsInRig()) {
    throw std::runtime_error("Shot " + id_ + " is not in a Rig.");
  }
  return rig_instance_.Value()->id;
}

RigCameraId Shot::GetRigCameraId() const {
  if (!IsInRig()) {
    throw std::runtime_error("Shot " + id_ + " is not in a Rig.");
  }
  return rig_camera_.Value()->id;
}

void ShotMeasurements::Set(const ShotMeasurements& other) {
  if (other.capture_time_.HasValue()) {
    capture_time_.SetValue(other.capture_time_.Value());
  } else {
    capture_time_.Reset();
  }
  if (other.gps_position_.HasValue()) {
    gps_position_.SetValue(other.gps_position_.Value());
  } else {
    gps_position_.Reset();
  }
  if (other.gps_accuracy_.HasValue()) {
    gps_accuracy_.SetValue(other.gps_accuracy_.Value());
  } else {
    gps_accuracy_.Reset();
  }
  if (other.compass_accuracy_.HasValue()) {
    compass_accuracy_.SetValue(other.compass_accuracy_.Value());
  } else {
    compass_accuracy_.Reset();
  }

  if (other.compass_angle_.HasValue()) {
    compass_angle_.SetValue(other.compass_angle_.Value());
  } else {
    compass_angle_.Reset();
  }
  if (other.accelerometer_.HasValue()) {
    accelerometer_.SetValue(other.accelerometer_.Value());
  } else {
    accelerometer_.Reset();
  }
  if (other.orientation_.HasValue()) {
    orientation_.SetValue(other.orientation_.Value());
  } else {
    orientation_.Reset();
  }
  if (other.sequence_key_.HasValue()) {
    sequence_key_.SetValue(other.sequence_key_.Value());
  } else {
    sequence_key_.Reset();
  }

  // Copy the attributes
  attributes_ = other.GetAttributes();
}

void Shot::RemoveLandmarkObservation(const FeatureId id) {
  auto* lm = landmark_id_.at(id);
  landmark_id_.erase(id);
  landmark_observations_.erase(lm);
}

void Shot::SetPose(const geometry::Pose& pose) {
  if (IsInRig()) {
    throw std::runtime_error(
        "Can't set the pose of Shot belonging to a RigInstance");
  }
  *pose_ = pose;
}

geometry::Pose Shot::GetPoseInRig() const {
  // pose(shot) = pose(rig_camera)*pose(instance)
  const auto& pose_instance = rig_instance_.Value()->GetPose();
  const auto& rig_camera_pose = rig_camera_.Value()->pose;
  return rig_camera_pose.Compose(pose_instance);
}

const geometry::Pose* const Shot::GetPose() const {
  if (IsInRig()) {
    *pose_ = GetPoseInRig();
  }
  return pose_.get();
}

geometry::Pose* const Shot::GetPose() {
  if (IsInRig()) {
    *pose_ = GetPoseInRig();
  }
  return pose_.get();
}

Vec2d Shot::Project(const Vec3d& global_pos) const {
  return shot_camera_->Project(GetPose()->RotationWorldToCamera() * global_pos +
                               GetPose()->TranslationWorldToCamera());
}

MatX2d Shot::ProjectMany(const MatX3d& points) const {
  MatX2d projected(points.rows(), 2);
  for (int i = 0; i < points.rows(); ++i) {
    projected.row(i) = Project(points.row(i));
  }
  return projected;
}

Vec3d Shot::Bearing(const Vec2d& point) const {
  return GetPose()->RotationCameraToWorld() * shot_camera_->Bearing(point);
}

MatX3d Shot::BearingMany(const MatX2d& points) const {
  MatX3d bearings(points.rows(), 3);
  for (int i = 0; i < points.rows(); ++i) {
    bearings.row(i) = Bearing(points.row(i));
  }
  return bearings;
}
}  // namespace map
