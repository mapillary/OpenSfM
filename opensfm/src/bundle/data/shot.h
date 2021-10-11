#pragma once

#include <bundle/data/camera.h>
#include <bundle/data/data.h>
#include <bundle/data/pose.h>

#include <unordered_set>

namespace bundle {

using RigCamera = Pose;

struct RigInstance : public Pose {
  RigInstance(const std::string &id, const geometry::Pose &value,
              const std::unordered_map<std::string, std::string> &shot_cameras)
      : Pose(id, value, Parametrization::CAM_TO_WORLD),
        shot_cameras(shot_cameras) {}

  foundation::OptionalValue<std::string> scale_group;
  std::unordered_map<std::string, std::string> shot_cameras;
};

struct Shot : public DataContainer {
  Shot(const std::string &id, bundle::Camera *camera, RigCamera *rig_camera,
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
