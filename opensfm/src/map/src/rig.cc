#include <map/rig.h>

namespace map {

std::set<map::ShotId> RigInstance::GetShotIDs() const {
  std::set<map::ShotId> shot_keys;
  std::transform(shots_.begin(), shots_.end(),
                 std::inserter(shot_keys, shot_keys.end()),
                 [](auto pair) { return pair.first; });
  return shot_keys;
}

void RigInstance::AddShot(map::RigCamera* rig_camera, map::Shot* shot) {
  const auto it_exist = std::find_if(
      shots_rig_cameras_.begin(), shots_rig_cameras_.end(),
      [&rig_camera](const auto& p) { return p.second->id == rig_camera->id; });
  if (it_exist != shots_rig_cameras_.end()) {
    throw std::runtime_error(rig_camera->id + " already exist in RigInstance");
  }

  if (rig_camera->relative_type == RigCamera::RelativeType::VARIABLE) {
    // TODO : never used/tested. Need to check implication wrt. Shot's RigCamera
    // pointer
    own_rig_cameras_[rig_camera->id].SetValue(*rig_camera);
    rig_camera = &own_rig_cameras_.at(rig_camera->id).Value();
  }

  shot->SetRig(this, rig_camera);
  shots_[shot->id_] = shot;
  shots_rig_cameras_[shot->id_] = rig_camera;
}

void RigInstance::UpdateInstancePoseWithShot(const map::ShotId& shot_id,
                                             const geometry::Pose& shot_pose) {
  const auto it_exist = shots_rig_cameras_.find(shot_id);
  if (it_exist == shots_rig_cameras_.end()) {
    throw std::runtime_error("Cannot find " + shot_id + " in RigInstance");
  }

  // pose(shot) = pose(rig_camera)*pose(instance)
  const auto& reference_shot_pose = shot_pose;
  const auto& rig_camera_pose = shots_rig_cameras_.at(shot_id)->pose;

  // Update instance
  pose_ = rig_camera_pose.Inverse().Compose(reference_shot_pose);
}

void RigInstance::UpdateRigCameraPose(const map::RigCameraId& rig_camera_id,
                                      const geometry::Pose& pose) {
  const auto it_exist =
      std::find_if(shots_rig_cameras_.begin(), shots_rig_cameras_.end(),
                   [&rig_camera_id](const auto& p) {
                     return p.second->id == rig_camera_id;
                   });
  if (it_exist == shots_rig_cameras_.end()) {
    throw std::runtime_error(rig_camera_id + " doesn't exist in RigInstance");
  }
  it_exist->second->pose = pose;
}
}  // namespace map
