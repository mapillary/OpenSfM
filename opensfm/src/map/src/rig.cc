#include <map/rig.h>

namespace map {

RigCamera& RigModel::GetRigCamera(const map::RigCameraId& rig_camera_id) {
  auto it_exist = rig_cameras_.find(rig_camera_id);
  if (it_exist == rig_cameras_.end()) {
    throw std::runtime_error(rig_camera_id + " doesn't exists in RigModel");
  }
  return it_exist->second;
}

const RigCamera& RigModel::GetRigCamera(
    const map::RigCameraId& rig_camera_id) const {
  const auto it_exist = rig_cameras_.find(rig_camera_id);
  if (it_exist == rig_cameras_.end()) {
    throw std::runtime_error(rig_camera_id + " doesn't exists in RigModel");
  }
  return it_exist->second;
}

std::set<map::ShotId> RigInstance::GetShotIDs() const {
  std::set<map::ShotId> shot_keys;
  std::transform(shots_.begin(), shots_.end(),
                 std::inserter(shot_keys, shot_keys.end()),
                 [](auto pair) { return pair.first; });
  return shot_keys;
}

void RigInstance::AddShot(const map::RigCameraId& rig_camera_id,
                          map::Shot* shot) {
  const auto it_exist = std::find_if(
      shots_rig_cameras_.begin(), shots_rig_cameras_.end(),
      [&rig_camera_id](const auto& p) { return p.second == rig_camera_id; });
  if (it_exist != shots_rig_cameras_.end()) {
    throw std::runtime_error(rig_camera_id + " already exist in RigInstance");
  }

  const RigCamera* rig_camera = &rig_model_->GetRigCamera(rig_camera_id);
  shot->SetRig(this, rig_camera);
  shots_[shot->id_] = shot;
  shots_rig_cameras_[shot->id_] = rig_camera->id;
}

void RigInstance::UpdateInstancePoseWithShot(const map::ShotId& shot_id,
                                             const geometry::Pose& shot_pose) {
  const auto it_exist = shots_rig_cameras_.find(shot_id);
  if (it_exist == shots_rig_cameras_.end()) {
    throw std::runtime_error("Cannot find " + shot_id + " in RigInstance");
  }

  // pose(shot) = pose(rig_camera)*pose(instance)
  const auto& reference_shot_pose = shot_pose;
  const auto& rig_camera_pose = rig_model_->GetRigCamera(it_exist->second).pose;

  // Update instance
  pose_ = rig_camera_pose.Inverse().Compose(reference_shot_pose);
}

void RigInstance::UpdateRigCameraPose(const map::RigCameraId& rig_camera_id,
                                      const geometry::Pose& pose) {
  rig_model_->GetRigCamera(rig_camera_id).pose = pose;
}
}  // namespace map
