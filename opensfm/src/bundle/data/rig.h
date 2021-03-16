#pragma once

#include <bundle/data/camera.h>
#include <bundle/data/data.h>
#include <bundle/data/pose.h>

namespace bundle {

using RigCamera = Pose;

struct RigModel : public DataContainer {
  RigModel(
      const std::string &id,
      const std::unordered_map<std::string, geometry::Pose> &rig_cameras_poses,
      const std::unordered_map<std::string, geometry::Pose>
          &rig_cameras_poses_prior,
      const geometry::Pose &sigma)
      : DataContainer(id) {
    for (const auto &rig_camera : rig_cameras_poses) {
      const auto rig_camera_id = rig_camera.first;
      const auto &prior = rig_cameras_poses_prior.at(rig_camera_id);
      auto &rig_camera_data =
          rig_cameras_
              .emplace(std::piecewise_construct,
                       std::forward_as_tuple(rig_camera_id),
                       std::forward_as_tuple(rig_camera_id, rig_camera.second,
                                             prior, sigma))
              .first->second;
      RegisterData(rig_camera_id, &rig_camera_data);
    }
  }
  RigCamera *GetRigCamera(const std::string &rig_camera_id) {
    return static_cast<RigCamera *>(GetData(rig_camera_id));
  }

  std::unordered_map<std::string, RigCamera *> GetRigCameras() {
    std::unordered_map<std::string, RigCamera *> cameras;
    for (const auto &c : rig_cameras_) {
      cameras[c.first] = GetRigCamera(c.first);
    }
    return cameras;
  }

 private:
  std::unordered_map<std::string, RigCamera> rig_cameras_;
};

using RigInstance = Pose;

struct RigShot : public DataContainer {
  RigShot(const std::string &id, bundle::Camera *camera, RigCamera *rig_camera,
          RigInstance *rig_instance)
      : DataContainer(id) {
    RegisterData("camera", camera);
    RegisterData("rig_camera", rig_camera);
    RegisterData("rig_instance", rig_instance);
  }

  bundle::Camera *GetCamera() {
    return static_cast<bundle::Camera *>(GetData("camera"));
  }
  RigCamera *GetRigCamera() {
    return static_cast<RigCamera *>(GetData("rig_camera"));
  }
  RigInstance *GetRigInstance() {
    return static_cast<RigInstance *>(GetData("rig_instance"));
  }
};
}  // namespace bundle
