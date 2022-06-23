#include <map/landmark.h>
#include <map/rig.h>
#include <map/shot.h>

#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <string>
namespace {
bool IsSingleShotRig(const map::RigInstance* rig_instance,
                     const map::RigCamera* rig_camera) {
  const bool has_identity_rig_camera = rig_camera->pose.IsIdentity();
  const bool is_single_shot_instance = rig_instance->NumberOfShots() == 1;
  return has_identity_rig_camera && is_single_shot_instance;
}
}  // namespace

namespace map {

Shot::Shot(const ShotId& shot_id, const geometry::Camera* const shot_camera,
           RigInstance* rig_instance, RigCamera* rig_camera,
           const geometry::Pose& pose)
    : id_(shot_id),
      pose_(std::make_unique<geometry::Pose>(pose)),
      rig_instance_(rig_instance),
      rig_camera_(rig_camera),
      shot_camera_(shot_camera) {
  rig_instance_->AddShot(rig_camera_, this);
  rig_instance_->UpdateInstancePoseWithShot(shot_id, pose);
}

Shot::Shot(const ShotId& shot_id, const geometry::Camera* const shot_camera,
           RigInstance* rig_instance, RigCamera* rig_camera)
    : id_(shot_id),
      pose_(std::make_unique<geometry::Pose>(geometry::Pose())),
      rig_instance_(rig_instance),
      rig_camera_(rig_camera),
      shot_camera_(shot_camera) {
  rig_instance_->AddShot(rig_camera_, this);
}

Shot::Shot(const ShotId& shot_id, const geometry::Camera& shot_camera,
           const geometry::Pose& pose)
    : id_(shot_id),
      pose_(std::make_unique<geometry::Pose>(pose)),
      own_rig_instance_(map::RigInstance(shot_id)),
      own_rig_camera_(map::RigCamera(geometry::Pose(), shot_id)),
      rig_instance_(&own_rig_instance_.Value()),
      rig_camera_(&own_rig_camera_.Value()),
      own_camera_(shot_camera),
      shot_camera_(&own_camera_.Value()) {
  rig_instance_->AddShot(rig_camera_, this);
  rig_instance_->SetPose(pose);
}

bool Shot::IsInRig() const { return true; }

void Shot::SetRig(RigInstance* rig_instance, RigCamera* rig_camera) {
  rig_instance_ = rig_instance;
  rig_camera_ = rig_camera;
  pose_ = std::make_unique<geometry::PoseImmutable>(*pose_);
}

const RigInstanceId& Shot::GetRigInstanceId() const {
  return rig_instance_->id;
}

const RigCameraId& Shot::GetRigCameraId() const { return rig_camera_->id; }

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
  if (other.opk_angles_.HasValue()) {
    opk_angles_.SetValue(other.opk_angles_.Value());
  } else {
    opk_angles_.Reset();
  }
  if (other.opk_accuracy_.HasValue()) {
    opk_accuracy_.SetValue(other.opk_accuracy_.Value());
  } else {
    opk_accuracy_.Reset();
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
  if (other.gravity_down_.HasValue()) {
    gravity_down_.SetValue(other.gravity_down_.Value());
  } else {
    gravity_down_.Reset();
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
  const auto find_feature = landmark_id_.find(id);
  if (find_feature == landmark_id_.end()) {
    throw std::runtime_error("Can't find Feature ID " + std::to_string(id) +
                             " in Shot " + this->id_);
  }
  auto* lm = find_feature->second;
  landmark_id_.erase(id);
  landmark_observations_.erase(lm);
}

void Shot::SetPose(const geometry::Pose& pose) {
  if (!IsSingleShotRig(rig_instance_, rig_camera_)) {
    throw std::runtime_error(
        "Can't set the pose of Shot belonging to a RigInstance");
  } else {
    rig_instance_->SetPose(pose);
  }
  *pose_ = pose;
}

geometry::Pose Shot::GetPoseInRig() const {
  // pose(shot) = pose(rig_camera)*pose(instance)
  const auto& pose_instance = rig_instance_->GetPose();
  const auto& rig_camera_pose = rig_camera_->pose;
  return rig_camera_pose.Compose(pose_instance);
}

const geometry::Pose* const Shot::GetPose() const {
  *pose_ = GetPoseInRig();
  if (IsSingleShotRig(rig_instance_, rig_camera_)) {
    return &rig_instance_->GetPose();
  }
  return pose_.get();
}

geometry::Pose* const Shot::GetPose() {
  *pose_ = GetPoseInRig();
  if (IsSingleShotRig(rig_instance_, rig_camera_)) {
    return &rig_instance_->GetPose();
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
