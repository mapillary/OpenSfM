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
  /* Pose of the camera wrt. the rig coordinate frame */
  geometry::Pose pose;

  /* Unique identifier of this RigCamera inside the RigModel */
  map::RigCameraId id;

  RigCamera() = default;
  RigCamera(const geometry::Pose& pose, const map::RigCameraId& id)
      : pose(pose), id(id) {}
};

struct RigModel {
  enum RelativeType {
    // All instances of this rig have their relatives poses being held
    FIXED = 0,
    // All instances shares the same relatives poses which are optimized
    SHARED = 1,
    // Each rig instance has its own relative pose wich is optimized
    VARIABLE = 2,
  };

  // Specify how to optimize rig relatives
  RelativeType relative_type{RelativeType::SHARED};

  // Unique identifier of this rig
  map::RigModelId id;

  RigModel() = default;
  explicit RigModel(const map::RigModelId& id) : id(id) {}

  // RigCameras
  void AddRigCamera(const RigCamera& rig_camera) {
    rig_cameras_[rig_camera.id] = rig_camera;
  }
  RigCamera& GetRigCamera(const map::RigCameraId& rig_camera_id);
  const RigCamera& GetRigCamera(const map::RigCameraId& rig_camera_id) const;
  const std::unordered_map<map::RigCameraId, RigCamera>& GetRigCameras() const {
    return rig_cameras_;
  }
  std::unordered_map<map::RigCameraId, RigCamera>& GetRigCameras() {
    return rig_cameras_;
  }

 private:
  // RigCameras that constitues that rig
  std::unordered_map<map::RigCameraId, RigCamera> rig_cameras_;
};

class RigInstance {
 public:
  map::RigInstanceId id;

  RigInstance(RigModel* rig_model, const RigInstanceId instance_id)
      : id(instance_id), rig_model_(rig_model) {
    if (rig_model->relative_type == RigModel::RelativeType::VARIABLE) {
      own_rig_model_.SetValue(*rig_model);
      rig_model_ = &own_rig_model_.Value();
    }
  }

  // Getters
  std::unordered_map<map::ShotId, map::Shot*> GetShots() const {
    return shots_;
  }
  std::unordered_map<map::ShotId, map::Shot*>& GetShots() { return shots_; }
  RigModel* GetRigModel() { return rig_model_; }
  const RigModel* GetRigModel() const { return rig_model_; }
  std::set<map::ShotId> GetShotIDs() const;
  std::unordered_map<map::ShotId, map::RigCameraId> GetShotRigCameraIDs()
      const {
    return shots_rig_cameras_;
  }

  // Pose
  geometry::Pose GetPose() const { return pose_; }
  geometry::Pose& GetPose() { return pose_; }
  void SetPose(const geometry::Pose& pose) { pose_ = pose; }

  // Add a new shot to this instance
  void AddShot(const map::RigCameraId& rig_camera_id, map::Shot* shot);

  // Update instance pose and shot's poses wrt. to a given shot of the instance
  void UpdateInstancePoseWithShot(const map::ShotId& shot_id,
                                  const geometry::Pose& shot_pose);

  // Update pose of this instance's RigCamera
  void UpdateRigCameraPose(const map::RigCameraId& rig_camera_id,
                           const geometry::Pose& pose);

 private:
  // Actual instanciation of a rig : each shot gets mapped to some RigCamera
  std::unordered_map<map::ShotId, map::Shot*> shots_;
  std::unordered_map<map::ShotId, map::RigCameraId> shots_rig_cameras_;

  // Pose of this rig coordinate frame wrt. world coordinates
  geometry::Pose pose_;

  // Model it instanciates
  RigModel* rig_model_;

  // For variable Rigs, each instance has its own copy
  foundation::OptionalValue<RigModel> own_rig_model_;
};
}  // namespace map
