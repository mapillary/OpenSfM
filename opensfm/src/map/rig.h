#pragma once

#include <geometry/camera.h>
#include <geometry/pose.h>
#include <map/defines.h>
#include <map/shot.h>

#include <exception>
#include <set>
#include <unordered_map>

namespace map {
struct RigCamera {
  enum RelativeType {
    // All instances of this rig camera have their relatives poses being held
    FIXED = 0,
    // All instances shares the same relative poses which are optimized
    SHARED = 1,
    // Each rig camera has its own pose wich is optimized
    VARIABLE = 2,
  };

  // Specify how to optimize rig relatives
  RelativeType relative_type{RelativeType::SHARED};

  /* Pose of the camera wrt. the rig coordinate frame */
  geometry::Pose pose;

  /* Unique identifier of this RigCamera */
  map::RigCameraId id;

  RigCamera() = default;
  RigCamera(const geometry::Pose& pose, const map::RigCameraId& id)
      : pose(pose), id(id) {}
};

class RigInstance {
 public:
  map::RigInstanceId id;

  explicit RigInstance(const RigInstanceId instance_id) : id(instance_id) {}

  // Getters
  std::unordered_map<map::ShotId, map::Shot*> GetShots() const {
    return shots_;
  }
  std::unordered_map<map::ShotId, map::Shot*>& GetShots() { return shots_; }
  std::unordered_map<map::ShotId, map::RigCamera*>& GetRigCameras() {
    return shots_rig_cameras_;
  }
  std::unordered_map<map::ShotId, map::RigCamera*> GetRigCameras() const {
    return shots_rig_cameras_;
  }
  std::set<map::ShotId> GetShotIDs() const;

  // Pose
  const geometry::Pose& GetPose() const { return pose_; }
  geometry::Pose& GetPose() { return pose_; }
  void SetPose(const geometry::Pose& pose) { pose_ = pose; }

  // Add a new shot to this instance
  void AddShot(map::RigCamera* rig_camera, map::Shot* shot);

  // Update instance pose and shot's poses wrt. to a given shot of the instance
  void UpdateInstancePoseWithShot(const map::ShotId& shot_id,
                                  const geometry::Pose& shot_pose);

  // Update pose of this instance's RigCamera
  void UpdateRigCameraPose(const map::RigCameraId& rig_camera_id,
                           const geometry::Pose& pose);

 private:
  // Actual instanciation of a rig : each shot gets mapped to some RigCamera
  std::unordered_map<map::ShotId, map::Shot*> shots_;
  std::unordered_map<map::ShotId, map::RigCamera*> shots_rig_cameras_;

  // Pose of this rig coordinate frame wrt. world coordinates
  geometry::Pose pose_;

  // For variable Rigs, each instance has its own copy of RigCameras
  std::unordered_map<map::ShotId, foundation::OptionalValue<map::RigCamera>>
      own_rig_cameras_;
};
}  // namespace map
